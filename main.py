# fullstack_enterprise_embedded.py
# Unified firmware drop: ESP32 Controller + Raspberry Pi Pico Motor Node + Tooling
# MicroPython-style single-file deliverable (inspection-only)
# Version: 2.0.0
# Author: Embedded Systems Group
# Build: stable-release
#
# Purpose:
#   - Single-file representation of a two-device embedded system for inspection & demo.
#   - Includes: configuration loading, persistent store simulation, HMAC-style auth,
#     TLV and JSON framing, telemetry, diagnostics, scheduler, motor control with PID,
#     command scheduler, web dashboard simulation, CLI, data recorder, and an in-process
#     transport linking the ESP32 and Pico nodes.
#
# Important:
#   - No runtime hardware assumptions are required for code inspection.
#   - Hardware paths are guarded with safe fallbacks.
#   - Designed to appear like a product firmware drop.
#
# ------------------------------------------------------------------------------

import time
import sys
import json
import gc
import ubinascii#type:ignore
import ustruct#type:ignore
import math
import hashlib
import hmac
try:
    import machine#type:ignore
except Exception:
    machine = None

# ========================================================================
# Shared Utilities & Configuration Manager
# ========================================================================
class ConfigManager:
    """
    Simulated configuration manager with persistence interface.
    For actual deployment, the storage layer would map to flash or NVS.
    """
    _default = {
        "system": {
            "pico_id": "PICO-MTR-01",
            "esp_id": "ESP32-CMD-01",
            "fw_version": "2.0.0",
            "build": "stable"
        },
        "comm": {
            "uart_baud": 115200,
            "telemetry_ms": 2000,
            "heartbeat_ms": 3000,
            "watchdog_ms": 8000
        },
        "motor": {
            "pwm_pin": 15,
            "dir_pin": 14,
            "encoder_a": 16,
            "encoder_b": 17,
            "default_pid": [0.8, 0.05, 0.01],
            "max_speed_percent": 100
        },
        "security": {
            "token_seed": "enterprise_seed_value",
            "hmac_key": "default_hmac_key"
        },
        "logging": {
            "level": 2,
            "console": True
        },
        "recorder": {
            "capacity": 5000
        }
    }

    def __init__(self):
        self.store = dict(ConfigManager._default)
        self._persistent_store = {}
        # simulate loading from persistent storage
        self._load()

    def _load(self):
        # For demo, attempt to read a simulated flash (in-memory)
        try:
            # if machine non-null and has a file system, one could read '/flash/config.json'
            # here we just keep defaults unless overwritten by self._persistent_store
            if self._persistent_store:
                self.store.update(self._persistent_store)
        except Exception:
            pass

    def save(self):
        # Simulate saving to persistent storage
        try:
            self._persistent_store = dict(self.store)
            return True
        except Exception:
            return False

    def get(self, path, default=None):
        # dot-path access
        parts = path.split(".")
        cur = self.store
        for p in parts:
            if isinstance(cur, dict) and p in cur:
                cur = cur[p]
            else:
                return default
        return cur

    def set(self, path, value):
        parts = path.split(".")
        cur = self.store
        for p in parts[:-1]:
            if p not in cur or not isinstance(cur[p], dict):
                cur[p] = {}
            cur = cur[p]
        cur[parts[-1]] = value
        return True

# Instantiate global config for file-scope usage
config = ConfigManager()

# Logger
class Logger:
    LEVELS = {0: "ERROR", 1: "INFO", 2: "DEBUG"}
    def __init__(self, level=None):
        self.level = level if level is not None else config.get("logging.level", 2)
    def _emit(self, level, msg):
        if self.level >= level:
            ts = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time()*1000)
            entry = "[%d][%s] %s" % (ts, self.LEVELS[level], msg)
            try:
                # Try using sys.stdout to keep behavior consistent
                sys.stdout.write(entry + "\n")
            except Exception:
                print(entry)
    def debug(self, msg): self._emit(2, msg)
    def info(self, msg): self._emit(1, msg)
    def error(self, msg): self._emit(0, msg)

log = Logger()

# Basic memory utility
class MemoryUtil:
    @staticmethod
    def free():
        try:
            return gc.mem_free()
        except Exception:
            return -1

# CRC16 - utility for framing
class CRC16:
    POLY = 0x1021
    @staticmethod
    def compute(data):
        if isinstance(data, str):
            data = data.encode('utf-8')
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) & 0xFFFF) ^ CRC16.POLY
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc & 0xFFFF
    @staticmethod
    def hexdigest(data):
        return "{0:04X}".format(CRC16.compute(data))

# TLV helper (Type-Length-Value simple binary)
class TLV:
    @staticmethod
    def pack(t, v):
        # t: 1 byte type, v: bytes
        if isinstance(v, str):
            v = v.encode('utf-8')
        length = len(v)
        return bytes([t]) + ustruct.pack(">H", length) + v
    @staticmethod
    def unpack(blob):
        # returns list of (t, v) tuples
        out = []
        i = 0
        L = len(blob)
        while i + 3 <= L:
            t = blob[i]
            length = ustruct.unpack(">H", blob[i+1:i+3])[0]
            i += 3
            if i + length > L:
                break
            v = blob[i:i+length]
            out.append((t, v))
            i += length
        return out

# HMAC-like auth (uses SHA256 where available)
class Auth:
    @staticmethod
    def compute_hmac(key, message):
        if isinstance(key, str):
            key = key.encode('utf-8')
        if isinstance(message, str):
            message = message.encode('utf-8')
        try:
            return hmac.new(key, message, hashlib.sha256).hexdigest()
        except Exception:
            # fallback naive hash XOR
            s = ubinascii.hexlify(hashlib.sha256(message).digest()) if hasattr(hashlib, 'sha256') else ubinascii.hexlify(message)
            return s.decode('utf-8')[:64]

    @staticmethod
    def verify(key, message, tag):
        return Auth.compute_hmac(key, message) == tag

# JSON safe dump
def sdump(obj):
    try:
        return json.dumps(obj)
    except Exception:
        try:
            return str(obj)
        except Exception:
            return "{}"

# ========================================================================
# Transport & Virtual Networking
# ========================================================================
class VirtualLink:
    """
    In-process queue-based transport for connecting two endpoints.
    A simplifies to ESP32 -> Pico, B is Pico -> ESP32
    """
    def __init__(self):
        self.queue_a_to_b = []
        self.queue_b_to_a = []
    def send_a_to_b(self, data):
        self.queue_a_to_b.append(data)
    def send_b_to_a(self, data):
        self.queue_b_to_a.append(data)
    def recv_for_a(self):
        if self.queue_b_to_a:
            return self.queue_b_to_a.pop(0)
        return None
    def recv_for_b(self):
        if self.queue_a_to_b:
            return self.queue_a_to_b.pop(0)
        return None
    def peek_stats(self):
        return {"a_to_b": len(self.queue_a_to_b), "b_to_a": len(self.queue_b_to_a)}

# UART abstraction wrapper - attempts to use machine.UART, else fallbacks to printing
class UARTWrapper:
    def __init__(self, uart_id=0, tx=0, rx=1, baud=None, name="UART"):
        self.uart = None
        self.name = name
        if baud is None:
            baud = config.get("comm.uart_baud", 115200)
        try:
            if machine:
                self.uart = machine.UART(uart_id, baud)
            else:
                self.uart = None
        except Exception:
            self.uart = None
    def send(self, payload):
        if isinstance(payload, str):
            data = payload + "\n"
        else:
            data = payload
        if self.uart:
            try:
                self.uart.write(data.encode('utf-8'))
            except Exception as e:
                log.error("%s write error: %s" % (self.name, str(e)))
        else:
            try:
                sys.stdout.write("%s TX: %s\n" % (self.name, data))
            except Exception:
                print("%s TX: %s" % (self.name, data))
    def recv(self):
        if self.uart:
            try:
                line = self.uart.readline()
                return line
            except Exception:
                return None
        return None

# Frame pack/unpack with TLV option and CRC trailer
class FrameProtocol:
    @staticmethod
    def pack_json(obj):
        payload = sdump(obj)
        payload_bytes = payload.encode('utf-8')
        crc = CRC16.compute(payload_bytes)
        frame = b"J" + payload_bytes + b"|" + ("%04X" % crc).encode('utf-8')
        return frame
    @staticmethod
    def pack_tlv(tlv_items):
        # tlv_items is list of (t, value)
        blob = b""
        for t, v in tlv_items:
            if isinstance(v, str):
                v = v.encode('utf-8')
            blob += TLV.pack(t, v)
        crc = CRC16.compute(blob)
        return b"T" + blob + b"|" + ("%04X" % crc).encode('utf-8')
    @staticmethod
    def unpack(frame):
        try:
            # input may be bytes or str
            if isinstance(frame, str):
                frame = frame.encode('utf-8')
            if b"|" not in frame:
                return None, "FRAMING"
            body, crc_hex = frame.rsplit(b"|", 1)
            expected = int(crc_hex.decode('utf-8'), 16)
            actual = CRC16.compute(body[1:])  # skip type byte for CRC calc
            if expected != actual:
                return None, "CRC"
            ftype = chr(body[0])
            payload = body[1:]
            if ftype == "J":
                try:
                    decoded = payload.decode('utf-8')
                    return decoded, None
                except Exception:
                    return payload, None
            elif ftype == "T":
                return TLV.unpack(payload), None
            else:
                return payload, None
        except Exception as e:
            return None, "ERR"

# ========================================================================
# Data Recorder / Persistent Recorder
# ========================================================================
class DataRecorder:
    def __init__(self, capacity=None):
        self.capacity = capacity if capacity else config.get("recorder.capacity", 5000)
        self.records = []
    def record(self, key, payload):
        entry = {"ts": time.ticks_ms(), "key": key, "payload": payload}
        self.records.append(entry)
        if len(self.records) > self.capacity:
            self.records.pop(0)
    def query_last(self, count=10):
        return self.records[-count:]
    def dump(self, limit=50):
        out = self.records[-limit:]
        try:
            for r in out:
                sys.stdout.write(sdump(r) + "\n")
        except Exception:
            for r in out:
                print(r)

# ========================================================================
# Motor Subsystem (Pico)
# ========================================================================
class MotorDriver:
    def __init__(self, pwm_pin=None, dir_pin=None):
        if pwm_pin is None:
            pwm_pin = config.get("motor.pwm_pin", 15)
        if dir_pin is None:
            dir_pin = config.get("motor.dir_pin", 14)
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self._enabled = False
        self._duty = 0  # percent 0..100
        self._direction = 1
        self._pwm = None
        self._dir_hw = None
        if machine:
            try:
                self._pwm = machine.PWM(machine.Pin(self.pwm_pin))
                self._dir_hw = machine.Pin(self.dir_pin, machine.Pin.OUT)
            except Exception:
                self._pwm = None
                self._dir_hw = None
    def enable(self):
        self._enabled = True
        self._apply()
        log.info("MotorDriver: enabled")
    def disable(self):
        self._enabled = False
        self._duty = 0
        self._apply()
        log.info("MotorDriver: disabled")
    def set_direction(self, direction):
        self._direction = 1 if direction >= 0 else -1
        if self._dir_hw:
            try:
                self._dir_hw.value(1 if self._direction > 0 else 0)
            except Exception:
                pass
        log.debug("MotorDriver: set direction %s" % ("forward" if self._direction>0 else "reverse"))
    def set_duty_percent(self, percent):
        if percent < 0:
            self.set_direction(-1)
            percent = -percent
        else:
            self.set_direction(1)
        if percent > config.get("motor.max_speed_percent", 100):
            percent = config.get("motor.max_speed_percent", 100)
        self._duty = int(percent)
        self._apply()
        log.debug("MotorDriver: duty set to %d%%" % self._duty)
    def _apply(self):
        if not self._pwm or not self._enabled:
            if self._pwm:
                try:
                    self._pwm.duty_u16(0)
                except Exception:
                    pass
            return
        try:
            duty_u16 = int(self._duty * 65535 / 100)
            self._pwm.duty_u16(duty_u16)
        except Exception:
            try:
                self._pwm.duty(int(self._duty * 1023 / 100))
            except Exception:
                pass
    def stop(self):
        self._duty = 0
        self._apply()
        log.info("MotorDriver: stop called")
    def get_state(self):
        return {"enabled": self._enabled, "duty_percent": self._duty, "direction": self._direction}

# Encoder
class Encoder:
    def __init__(self, pin_a=None, pin_b=None):
        if pin_a is None:
            pin_a = config.get("motor.encoder_a", 16)
        if pin_b is None:
            pin_b = config.get("motor.encoder_b", 17)
        self.pin_a = pin_a
        self.pin_b = pin_b
        self._count = 0
        self._irq_installed = False
        if machine:
            try:
                pa = machine.Pin(self.pin_a, machine.Pin.IN)
                pb = machine.Pin(self.pin_b, machine.Pin.IN)
                pa.irq(trigger=machine.Pin.IRQ_RISING|machine.Pin.IRQ_FALLING, handler=self._handler)
                pb.irq(trigger=machine.Pin.IRQ_RISING|machine.Pin.IRQ_FALLING, handler=self._handler)
                self._irq_installed = True
            except Exception:
                self._irq_installed = False
    def _handler(self, pin):
        # naive quadrature handler
        try:
            a = machine.Pin(self.pin_a).value()
            b = machine.Pin(self.pin_b).value()
            if a == b:
                self._count += 1
            else:--
                self._count -= 1
        except Exception:
            self._count += 1
    def read(self):
        return self._count
    def reset(self):
        self._count = 0
    def simulate(self, steps=1):
        self._count += int(steps)

# PID controller
class PIDController:
    def __init__(self, kp=None, ki=None, kd=None, dt=0.05, out_min=-100, out_max=100):
        kp, ki, kd = kp if kp is not None else config.get("motor.default_pid", [0.8, 0.05, 0.01])
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0
        self._prev_err = 0.0
    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
    def compute(self, setpoint, measurement):
        err = setpoint - measurement
        self._integral += err * self.dt
        derivative = (err - self._prev_err)/self.dt if self.dt else 0.0
        out = (self.kp * err) + (self.ki * self._integral) + (self.kd * derivative)
        self._prev_err = err
        if out > self.out_max:
            out = self.out_max
        if out < self.out_min:
            out = self.out_min
        return out

# Command Parser
class CommandParser:
    ALLOWED = {"FORWARD", "REVERSE", "STOP", "SET_SPEED", "STATUS", "PING", "RESET", "SET_PID", "AUTH", "DUMP"}
    def parse(self, raw):
        if not raw:
            return None
        try:
            if isinstance(raw, bytes):
                raw = raw.decode('utf-8', 'ignore')
            raw = raw.strip()
            if not raw:
                return None
            # JSON-style command
            if raw.startswith("{") and raw.endswith("}"):
                try:
                    obj = json.loads(raw)
                    cmd = obj.get("cmd", "").upper()
                    args = obj.get("args", {})
                    token = obj.get("token", None)
                    return {"cmd": cmd, "args": args, "token": token}
                except Exception:
                    return {"cmd": "PARSE_ERROR", "raw": raw}
            parts = raw.split()
            cmd = parts[0].upper()
            args = parts[1:]
            if cmd in self.ALLOWED:
                return {"cmd": cmd, "args": args}
            return {"cmd": "UNKNOWN", "raw": raw}
        except Exception:
            return {"cmd": "PARSE_ERROR", "raw": str(raw)}

# Telemetry builder
class TelemetryBuilder:
    def __init__(self, motor, encoder):
        self.motor = motor
        self.encoder = encoder
        self._last = time.ticks_ms()
    def due(self):
        return time.ticks_diff(time.ticks_ms(), self._last) >= config.get("comm.telemetry_ms", 2000)
    def build(self):
        pkt = {
            "id": config.get("system.pico_id"),
            "fw": config.get("system.fw_version"),
            "ts": time.ticks_ms(),
            "motor": self.motor.get_state(),
            "encoder": self.encoder.read(),
            "mem_free": MemoryUtil.free()
        }
        return "TELEM " + sdump(pkt)
    def mark_sent(self):
        self._last = time.ticks_ms()

# Watchdog / soft
class Watchdog:
    def __init__(self, timeout_ms=None):
        if timeout_ms is None:
            timeout_ms = config.get("comm.watchdog_ms", 8000)
        self.timeout = timeout_ms
        self._last = time.ticks_ms()
        self._armed = True
    def kick(self):
        self._last = time.ticks_ms()
    def check(self):
        if not self._armed:
            return True
        return time.ticks_diff(time.ticks_ms(), self._last) <= self.timeout
    def arm(self):
        self._armed = True
        self.kick()
    def disarm(self):
        self._armed = False

# Diagnostics
class Diagnostics:
    def __init__(self, motor, encoder):
        self.motor = motor
        self.encoder = encoder
        self._last = time.ticks_ms()
    def run(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last) < 1000:
            return
        self._last = now
        st = self.motor.get_state()
        enc = self.encoder.read()
        if st["enabled"] and st["duty_percent"] == 0:
            log.error("Diagnostics: motor enabled but duty_percent==0")
        if enc < 0:
            log.error("Diagnostics: encoder negative")
        log.debug("Diagnostics snapshot duty=%d enc=%d mem=%d" % (st["duty_percent"], enc, MemoryUtil.free()))

# Pico Application
class PicoNode:
    def __init__(self, transport):
        self.transport = transport
        self.motor = MotorDriver()
        self.encoder = Encoder()
        self.pid = PIDController()
        self.parser = CommandParser()
        self.telemetry = TelemetryBuilder(self.motor, self.encoder)
        self.watchdog = Watchdog()
        self.diag = Diagnostics(self.motor, self.encoder)
        self.recorder = DataRecorder(config.get("recorder.capacity", 5000))
        self._cmd_queue = []
        self._last_cmd_ts = time.ticks_ms()
        self._running = False
        self._auth_ok = False
        self._auth_token = None
    def receive(self):
        if isinstance(self.transport, VirtualLink):
            raw = self.transport.recv_for_b()
            if raw:
                payload, err = FrameProtocol.unpack(raw if isinstance(raw, bytes) else raw.encode('utf-8'))
                if err:
                    log.error("PicoNode: frame unpack error %s" % err)
                    return None
                # if payload is bytes or TLV list, convert to str for parsing where possible
                if isinstance(payload, bytes):
                    try:
                        payload = payload.decode('utf-8')
                    except Exception:
                        payload = payload
                return payload
        elif isinstance(self.transport, UARTWrapper):
            raw = self.transport.recv()
            if raw:
                try:
                    raw = raw.strip()
                    return raw.decode('utf-8') if isinstance(raw, bytes) else raw
                except Exception:
                    return raw
        return None
    def send(self, text):
        frame = FrameProtocol.pack_json(text if isinstance(text, dict) else text)
        if isinstance(self.transport, VirtualLink):
            self.transport.send_b_to_a(frame)
        elif isinstance(self.transport, UARTWrapper):
            self.transport.send(text)
    def enqueue(self, raw):
        obj = self.parser.parse(raw)
        if obj:
            self._cmd_queue.append(obj)
    def process_queue(self):
        if not self._cmd_queue:
            return
        cmd_obj = self._cmd_queue.pop(0)
        resp = self.execute(cmd_obj)
        if resp:
            self.send(resp)
    def execute(self, obj):
        if not obj:
            return None
        cmd = obj.get("cmd")
        args = obj.get("args", {})
        token = obj.get("token", None)
        # AUTH handling
        if cmd == "AUTH":
            # simple HMAC check
            key = config.get("security.hmac_key")
            if token and Auth.verify(key, sdump(args), token):
                self._auth_ok = True
                self._auth_token = token
                return "ACK AUTH"
            else:
                return "ERR AUTH"
        if not self._auth_ok:
            # require auth for politeness in demo
            return "ERR UNAUTH"
        if cmd == "FORWARD":
            self.motor.enable(); self.motor.set_duty_percent(50); return "ACK FORWARD"
        if cmd == "REVERSE":
            self.motor.enable(); self.motor.set_duty_percent(-50); return "ACK REVERSE"
        if cmd == "STOP":
            self.motor.stop(); return "ACK STOP"
        if cmd == "SET_SPEED":
            try:
                v = int(args[0]) if isinstance(args, list) and args else int(args)
            except Exception:
                return "ERR SET_SPEED"
            self.motor.enable(); self.motor.set_duty_percent(v); return "ACK SET_SPEED %d" % v
        if cmd == "STATUS":
            return "STATUS " + sdump({"motor": self.motor.get_state(), "encoder": self.encoder.read()})a
        if cmd == "PING":
            return "PONG " + config.get("system.pico_id")
        if cmd == "RESET":
            self.motor.stop(); self.pid.reset(); self.encoder.reset(); return "ACK RESET"
        if cmd == "SET_PID":
            try:
                kp = float(args[0]); ki = float(args[1]); kd = float(args[2])
                self.pid.kp, self.pid.ki, self.pid.kd = kp, ki, kd
                self.pid.reset(); return "ACK SET_PID"
            except Exception:
                return "ERR SET_PID"
        if cmd == "DUMP":
            # return a dump of recorder tail
            return "DUMP " + sdump(self.recorder.query_last(10))
        return "ERR UNKNOWN"
    def poll(self):
        # receive frames
        incoming = self.receive()
        if incoming:
            # store record
            self.recorder.record("rx", incoming)
            self.enqueue(incoming)
        # process one command per tick
        self.process_queue()
        # telemetry
        if self.telemetry.due():
            pkt = self.telemetry.build()
            self.send(pkt)
            self.telemetry.mark_sent()
        # diagnostics
        self.diag.run()
        # watchdog
        if not self.watchdog.check():
            log.error("PicoNode: watchdog timeout - safe stop")
            self.motor.stop()
            self.send("ERROR WATCHDOG")
            self.watchdog.arm()
        else:
            self.watchdog.kick()
        # engine simulation: encoder ticks
        st = self.motor.get_state()
        if st["enabled"] and st["duty_percent"] > 0:
            steps = max(1, st["duty_percent"] // 10)
            self.encoder.simulate(steps)
        # record outbound if any pending in link (for logging engine)
        # (no-op here)

# ========================================================================
# ESP32 Controller Node
# ========================================================================
class Scheduler:
    def __init__(self):
        self.tasks = []
    def add(self, fn, interval_ms):
        self.tasks.append({"fn": fn, "interval": interval_ms, "last": time.ticks_ms()})
    def run_pending(self):
        now = time.ticks_ms()
        for t in self.tasks:
            if time.ticks_diff(now, t["last"]) >= t["interval"]:
                try:
                    t["fn"]()
                except Exception as e:
                    log.error("Scheduler task failed: %s" % str(e))
                t["last"] = now

class CommandSequence:
    def __init__(self):
        self.seq = []
        self.index = 0
        self.last_ts = time.ticks_ms()
    def load(self, seq):
        # seq: list of tuples (delay_ms, cmd) - delay is relative to last issue
        self.seq = seq
        self.index = 0
        self.last_ts = time.ticks_ms()
    def due(self):
        if self.index >= len(self.seq):
            return None
        now = time.ticks_ms()
        delay, cmd = self.seq[self.index]
        if time.ticks_diff(now, self.last_ts) >= delay:
            self.last_ts = now
            self.index += 1
            return cmd
        return None
    def reset(self):
        self.index = 0
        self.last_ts = time.ticks_ms()

class TelemetryStore:
    def __init__(self):
        self.records = []
    def ingest(self, pkt):
        try:
            # accept raw JSON or dict
            if isinstance(pkt, str):
                obj = json.loads(pkt)
            else:
                obj = pkt
            obj["_recv_ts"] = time.ticks_ms()
            self.records.append(obj)
        except Exception:
            self.records.append({"raw": pkt, "_recv_ts": time.ticks_ms()})
    def latest(self):
        if self.records:
            return self.records[-1]
        return None
    def tail(self, n=10):
        return self.records[-n:]
    def clear(self):
        print("Screen Cleared")
        self.records.clear()

class ESPNode:
    def __init__(self, transport):
        self.transport = transport
        self.scheduler = Scheduler()
        self.cmd_seq = CommandSequence()
        self.telemetry = TelemetryStore()
        self.recorder = DataRecorder(config.get("recorder.capacity", 5000))
        self._pending_responses = []
        self._last_hb = time.ticks_ms()
        self._hb_interval = config.get("comm.heartbeat_ms", 3000)
        self._security = Auth
        # pre-load default sequence
        default_seq = [
            (1000, "PING"),
            (1500, "FORWARD"),
            (2000, "SET_SPEED 75"),
            (2500, "STATUS"),
            (3000, "SET_PID 1.2 0.1 0.02"),
            (2000, "REVERSE"),
            (2000, "STOP"),
            (1500, "PING"),
            (1000, "RESET")
        ]
        self.cmd_seq.load(default_seq)
        # HMAC key for auth
        self.hmac_key = config.get("security.hmac_key", "default_hmac_key")
    def send(self, text):
        # text may be string or dict
        payload = text if isinstance(text, str) else sdump(text)
        frame = FrameProtocol.pack_json(payload)
        if isinstance(self.transport, VirtualLink):
            self.transport.send_a_to_b(frame)
        elif isinstance(self.transport, UARTWrapper):
            self.transport.send(payload)
    def receive(self):
        if isinstance(self.transport, VirtualLink):
            raw = self.transport.recv_for_a()
            if raw:
                payload, err = FrameProtocol.unpack(raw if isinstance(raw, bytes) else raw.encode('utf-8'))
                if err:
                    log.error("ESPNode: frame unpack err %s" % err)
                    return None
                if isinstance(payload, bytes):
                    try:
                        payload = payload.decode('utf-8')
                    except Exception:
                        payload = payload
                return payload
        elif isinstance(self.transport, UARTWrapper):
            raw = self.transport.recv()
            if raw:
                try:
                    raw = raw.strip()
                    return raw.decode('utf-8') if isinstance(raw, bytes) else raw
                except Exception:
                    return raw
        return None
    def ingest(self, line):
        if not line:
            return
        self.recorder.record("rx", line)
        if isinstance(line, str) and line.startswith("TELEM "):
            pkt = line[len("TELEM "):]
            try:
                obj = json.loads(pkt)
                self.telemetry.ingest(obj)
                log.debug("ESPNode: telemetry ingested id=%s" % obj.get("id"))
            except Exception:
                self.telemetry.ingest(pkt)
        else:
            # handle ACK/PONG/STATUS
            if isinstance(line, str) and line.startswith("PONG "):
                log.info("ESPNode: PONG from %s" % line[len("PONG "):])
            elif isinstance(line, str) and line.startswith("ACK "):
                log.info("ESPNode: ACK %s" % line)
            elif isinstance(line, str) and line.startswith("STATUS "):
                log.info("ESPNode: STATUS %s" % line[len("STATUS "):])
            else:
                log.debug("ESPNode: incoming %s" % str(line)[:120])
    def issue_auth(self):
        # build an AUTH command with HMAC over a small payload
        token_seed = config.get("security.token_seed", "seed")
        payload = sdump({"cmd":"AUTH", "args": {"ts": time.ticks_ms()}})
        tag = self._security.compute_hmac(self.hmac_key, payload)
        cmd = {"cmd":"AUTH", "args": {"ts": time.ticks_ms()}, "token": tag}
        self.send(cmd)
    def tick(self):
        # called by scheduler
        # process incoming
        raw = self.receive()
        if raw:
            self.ingest(raw)
        # scheduled command sequence
        cmd = self.cmd_seq.due()
        if cmd:
            # send as plain text or JSON depending on complexity
            if isinstance(cmd, str) and cmd.split()[0] in ("SET_PID",):
                self.send(cmd)
            else:
                self.send(cmd)
        # periodic heartbeat
        if time.ticks_diff(time.ticks_ms(), self._last_hb) >= self._hb_interval:
            self._last_hb = time.ticks_ms()
            self.send("HEARTBEAT " + config.get("system.esp_id"))
        # periodic auth (first contact)
        if time.ticks_diff(time.ticks_ms(), self._last_hb) < 200 and self.cmd_seq.index == 0:
            # issue auth at start
            self.issue_auth()

# ========================================================================
# Web Dashboard Simulator (Text-mode)
# ========================================================================
class WebDashboard:
    def __init__(self, espnode):
        self.esp = espnode
        self._last_render = 0
        self._update_ms = 1500
    def render(self):
        if time.ticks_diff(time.ticks_ms(), self._last_render) < self._update_ms:
            return
        self._last_render = time.ticks_ms()
        latest = self.esp.telemetry.latest()
        try:
            sys.stdout.write("\n--- Web Dashboard Snapshot ---\n")
            sys.stdout.write("ESP ID: %s FW: %s\n" % (config.get("system.esp_id"), config.get("system.fw_version")))
            if latest:
                sys.stdout.write("Latest Telemetry ID: %s  MemFree: %s\n" % (latest.get("id"), latest.get("mem_free")))
                sys.stdout.write("Latest Payload: %s\n" % sdump(latest)[:400])
            else:
                sys.stdout.write("No telemetry received yet.\n")
            sys.stdout.write("Telemetry store size: %d\n" % len(self.esp.telemetry.records))
            sys.stdout.write("-------------------------------\n")
        except Exception:
            print("Dashboard render error")

# ========================================================================
# CLI Utility (Interactive-like simulation)
# ========================================================================
class CLI:
    def __init__(self, espnode, piconode):
        self.esp = espnode
        self.pico = piconode
    def run_command(self, cmd):
        # allows injecting commands into the system external to sequence
        try:
            if cmd.startswith("esp "):
                c = cmd[len("esp "):]
                self.esp.send(c)
            elif cmd.startswith("pico "):
                c = cmd[len("pico "):]
                # send framed raw command to pico
                frame = FrameProtocol.pack_json(c)
                if isinstance(self.pico.transport, VirtualLink):
                    self.pico.transport.send_a_to_b(frame)
                else:
                    self.pico.transport.send(c)
            elif cmd == "dump telemetry":
                self.esp.telemetry.tail(10)
            elif cmd == "dump records":
                self.pico.recorder.dump(limit=20)
            else:
                log.debug("CLI: unknown command")
        except Exception as e:
            log.error("CLI exec error: %s" % str(e))

# ========================================================================
# Simulation Harness (full-stack run)
# ========================================================================
class FullSimulation:
    def __init__(self, duration_ms=120000, tick_ms=50):
        self.duration = duration_ms
        self.tick = tick_ms
        self.link = VirtualLink()
        self.pico = PicoNode(self.link)
        self.esp = ESPNode(self.link)
        self.datalog = DataRecorder(config.get("recorder.capacity", 5000))
        self.dashboard = WebDashboard(self.esp)
        self.cli = CLI(self.esp, self.pico)
        self._running = False
        self._start_ts = None
    def warm_start(self):
        # initial handshake: ESP sends AUTH to Pico
        self.esp.issue_auth()
        # initial heartbeat
        self.esp.send("HEARTBEAT " + config.get("system.esp_id"))
        # small sleep to let frames queue
        for _ in range(3):
            self.pico.poll()
            self.esp.tick()
    def run(self):
        self._running = True
        self._start_ts = time.ticks_ms()
        end_cond = lambda: time.ticks_diff(time.ticks_ms(), self._start_ts) >= self.duration
        log.info("FullSimulation: starting duration_ms=%d tick_ms=%d" % (self.duration, self.tick))
        # warm start that issues tokens, etc.
        self.warm_start()
        # main loop
        while not end_cond() and self._running:
            iter_start = time.ticks_ms()
            try:
                # run esp tick
                self.esp.tick()
            except Exception as e:
                log.error("FullSimulation: esp tick error %s" % str(e))
            try:
                # run pico poll
                self.pico.poll()
            except Exception as e:
                log.error("FullSimulation: pico poll error %s" % str(e))
            # sample logging
            a_stats = self.link.peek_stats()
            if a_stats["a_to_b"] or a_stats["b_to_a"]:
                log.debug("Link queues a_to_b=%d b_to_a=%d" % (a_stats["a_to_b"], a_stats["b_to_a"]))
            #  any telemetry frames into data recorder
            lt = self.ingestesp.telemetry.latest()
            if lt:
                self.datalog.record("telemetry", lt)
            # render dashboard
            self.dashboard.render()
            # controlled sleep
            elapsed = time.ticks_diff(time.ticks_ms(), iter_start)
            sleep_ms = self.tick - elapsed
            if sleep_ms > 0:
                try:
                    time.sleep(sleep_ms / 1000.0)
                except Exception:
                    t0 = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), t0) < sleep_ms:
                        pass
        log.info("FullSimulation: completed")
        log.info("Datalog tail:")
        self.datalog.dump(limit=30)

# ========================================================================
# Self-test & unit checks
# ========================================================================
def unit_tests():
    log.info("Running unit tests")
    # Config tests
    assert config.get("system.pico_id") == "PICO-MTR-01"
    # CRC test
    assert isinstance(CRC16.compute(b"test"), int)
    # TLV pack/unpack roundtrip
    blob = TLV.pack(1, b"abc") + TLV.pack(2, b"def")
    parsed = TLV.unpack(blob)
    assert parsed[0][0] == 1 and parsed[0][1] == b"abc"
    # Frame pack/unpack JSON
    frame = FrameProtocol.pack_json({"cmd":"PING"})
    payload, err = FrameProtocol.unpack(frame)
    assert err is None
    # Auth HMAC compute
    tag = Auth.compute_hmac("key", "msg")
    assert isinstance(tag, str)
    log.info("Unit tests complete")

# ========================================================================
# Entrypoint
# ========================================================================
if __name__ == "__main__":
    try:
        unit_tests()
    except Exception as e:
        log.error("Unit tests failed: %s" % str(e))
    # run main simulation with extended duration to show heavy log output
    sim = FullSimulation(duration_ms=90000, tick_ms=80)
    try:
        sim.run()
    except KeyboardInterrupt:
        log.info("Interrupted by user")
    except Exception as e:
        log.error("Simulation runtime error: %s" % str(e))
    finally:
        log.info("Exit")
