// enhancements.cpp
// Small utility that "enhances" main.py by applying a tiny compatibility shim,
// normalizing line endings, and writing an enhanced copy to main_enhanced.py.
// Build: g++ -std=c++17 enhancements.cpp -o enhancements

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <filesystem>
#include <chrono>

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    const std::string input_name = "main.py";
    const std::string output_name = "main_enhanced.py";

    if (!fs::exists(input_name)) {
        std::cerr << "enhancements: input file '" << input_name << "' not found in current directory.\n";
        return 2;
    }

    std::ifstream in(input_name, std::ios::binary);
    if (!in) {
        std::cerr << "enhancements: failed to open '" << input_name << "' for reading.\n";
        return 3;
    }

    std::ostringstream ss;
    ss << in.rdbuf();
    std::string content = ss.str();
    in.close();

    // Normalize CRLF -> LF
    std::string::size_type pos = 0;
    while ((pos = content.find("\r\n", pos)) != std::string::npos) {
        content.replace(pos, 2, "\n");
        pos += 1;
    }

    // Build a small compatibility shim to place at top of file.
    std::ostringstream header;
    header << "# --- enhanced by enhancements.cpp ---\n";
    header << "# Added small compatibility shims and normalized line endings.\n";
    header << "# Generated: " << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << "\n\n";
    header << "try:\n";
    header << "    # MicroPython-specific modules fallback to stdlib where possible\n";
    header << "    import ubinascii as _ubinascii\n";
    header << "except Exception:\n";
    header << "    import binascii as _ubinascii\n";
    header << "\n";

    // Prepend header unless file already contains the exact header marker.
    const std::string marker = "# --- enhanced by enhancements.cpp ---";
    if (content.find(marker) == std::string::npos) {
        content = header.str() + content;
    }

    std::ofstream out(output_name, std::ios::binary);
    if (!out) {
        std::cerr << "enhancements: failed to open '" << output_name << "' for writing.\n";
        return 4;
    }
    out << content;
    out.close();

    std::cout << "enhancements: wrote '" << output_name << "' (" << fs::file_size(output_name) << " bytes).\n";
    return 0;
}
