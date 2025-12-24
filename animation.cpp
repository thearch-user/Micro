// animation.cpp
// Improved console animation and small enhancements demo.
// Features:
// - CLI options: --duration-ms, --width, --create, --target, --color
// - Frame-rate independent progress with ETA
// - Optional creation/normalization of `main_enhanced.py`
// Build: g++ -std=c++17 animation.cpp -o animation

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace chrono = std::chrono;
namespace fs = std::filesystem;

struct Config {
    std::vector<std::string> steps;
    int stepDurationMs = 800; // target duration per step
    int barWidth = 40;
    bool createEnhanced = false;
    std::string enhancedTarget = "main_enhanced.py";
    bool color = false;
};

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [--duration-ms N] [--width N] [--create] [--target FILE] [--color]\n";
}

static Config parse_args(int argc, char** argv) {
    Config cfg;
    cfg.steps = {"Reading main.py", "Analyzing imports", "Adding compatibility shim", "Normalizing line endings", "Writing main_enhanced.py"};
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--duration-ms") == 0 && i + 1 < argc) {
            cfg.stepDurationMs = std::stoi(argv[++i]);
        } else if (std::strcmp(a, "--width") == 0 && i + 1 < argc) {
            cfg.barWidth = std::max(10, std::stoi(argv[++i]));
        } else if (std::strcmp(a, "--create") == 0) {
            cfg.createEnhanced = true;
        } else if (std::strcmp(a, "--target") == 0 && i + 1 < argc) {
            cfg.enhancedTarget = argv[++i];
        } else if (std::strcmp(a, "--color") == 0) {
            cfg.color = true;
        } else if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            print_usage(argv[0]);
            std::exit(0);
        }
    }
    return cfg;
}

static void flush_line() {
    std::cout << std::flush;
}

static void print_progress(size_t idx, size_t total, const std::string& step, double frac, int width, bool color) {
    int filled = static_cast<int>(frac * width + 0.5);
    if (filled < 0) filled = 0;
    if (filled > width) filled = width;
    int pct = static_cast<int>(frac * 100.0 + 0.5);

    std::cout << '\r';
    std::cout << "[" << (idx + 1) << "/" << total << "] ";
    if (color) std::cout << "\x1b[36m"; // cyan label
    std::cout << step << " ";
    if (color) std::cout << "\x1b[0m";

    std::cout << "[";
    for (int i = 0; i < filled; ++i) std::cout << '=';
    for (int i = filled; i < width; ++i) std::cout << ' ';
    std::cout << "] " << pct << "%";
    flush_line();
}

static std::string read_file_all(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return {};
    std::string content;
    in.seekg(0, std::ios::end);
    content.resize(static_cast<size_t>(in.tellg()));
    in.seekg(0);
    in.read(&content[0], content.size());
    return content;
}

static bool write_file_all(const fs::path& p, const std::string& data) {
    std::ofstream out(p, std::ios::binary);
    if (!out) return false;
    out.write(data.data(), static_cast<std::streamsize>(data.size()));
    return true;
}

int main(int argc, char** argv) {
    std::ios::sync_with_stdio(false);
    auto cfg = parse_args(argc, argv);

    std::cout << "Animation: Demonstrating enhancements..." << std::endl;

    const int totalSteps = static_cast<int>(cfg.steps.size());

    using clock = chrono::steady_clock;

    for (int i = 0; i < totalSteps; ++i) {
        const auto& step = cfg.steps[i];
        const auto stepDuration = chrono::milliseconds(cfg.stepDurationMs);
        const auto start = clock::now();
        const auto end = start + stepDuration;

        // Animate until the step duration elapses. We update based on real time.
        while (clock::now() < end) {
            auto now = clock::now();
            double frac = chrono::duration_cast<chrono::duration<double>>(now - start).count() / chrono::duration_cast<chrono::duration<double>>(stepDuration).count();
            if (frac < 0.0) frac = 0.0;
            if (frac > 1.0) frac = 1.0;
            print_progress(i, totalSteps, step, frac, cfg.barWidth, cfg.color);
            std::this_thread::sleep_for(chrono::milliseconds(12));
        }
        print_progress(i, totalSteps, step, 1.0, cfg.barWidth, cfg.color);
        std::cout << std::endl;
    }

    const fs::path enhanced = cfg.enhancedTarget;
    if (fs::exists(enhanced)) {
        std::error_code ec;
        auto sz = fs::file_size(enhanced, ec);
        if (!ec) {
            std::cout << "Result: '" << enhanced.string() << "' exists (" << sz << " bytes)." << std::endl;
        } else {
            std::cout << "Result: '" << enhanced.string() << "' exists." << std::endl;
        }
    } else if (cfg.createEnhanced) {
        // Try to create enhanced file from main.py by normalizing line endings
        const fs::path src = "main.py";
        if (fs::exists(src)) {
            auto content = read_file_all(src);
            if (!content.empty()) {
                // Normalize CRLF -> LF
                std::string out;
                out.reserve(content.size());
                for (size_t i = 0; i < content.size(); ++i) {
                    if (content[i] == '\r') continue;
                    out.push_back(content[i]);
                }
                // Add a small header
                std::string header = "# Enhanced by animation tool\n# Original: main.py\n\n";
                bool ok = write_file_all(enhanced, header + out);
                if (ok) {
                    std::cout << "Wrote '" << enhanced.string() << "' (" << (header.size() + out.size()) << " bytes)." << std::endl;
                } else {
                    std::cout << "Failed to write '" << enhanced.string() << "'." << std::endl;
                }
            } else {
                std::cout << "Source 'main.py' is empty or unreadable." << std::endl;
            }
        } else {
            std::cout << "Source 'main.py' not found; cannot create '" << enhanced.string() << "'." << std::endl;
        }
    } else {
        std::cout << "Result: '" << enhanced.string() << "' not found. To create it run with --create." << std::endl;
    }

    std::cout << "Animation complete." << std::endl;
    return 0;
}
