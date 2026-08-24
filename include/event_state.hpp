#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

namespace event_state {

inline void syncFile(const std::filesystem::path& file) {
#ifndef _WIN32
    const int descriptor = ::open(file.string().c_str(), O_WRONLY);
    if (descriptor >= 0) {
        ::fsync(descriptor);
        ::close(descriptor);
    }
#else
    (void)file;
#endif
}

inline void syncDirectory(const std::filesystem::path& directory) {
#ifndef _WIN32
    const int descriptor = ::open(directory.string().c_str(), O_RDONLY | O_DIRECTORY);
    if (descriptor >= 0) {
        ::fsync(descriptor);
        ::close(descriptor);
    }
#else
    (void)directory;
#endif
}

inline bool writeAtomically(const std::filesystem::path& target, const std::string& content) {
    std::error_code error;
    std::filesystem::create_directories(target.parent_path(), error);
    if (error) {
        return false;
    }
    const auto temporary = target.string() + ".tmp";
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
        return false;
    }
    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.close();
    if (!output) {
        std::filesystem::remove(temporary, error);
        return false;
    }
    syncFile(temporary);
    std::filesystem::rename(temporary, target, error);
    if (error) {
        // Windows does not replace an existing destination. State transitions only
        // overwrite diagnostic markers, so remove-and-rename is an acceptable fallback.
        std::error_code removeError;
        std::filesystem::remove(target, removeError);
        error.clear();
        std::filesystem::rename(temporary, target, error);
    }
    if (error) {
        std::filesystem::remove(temporary, error);
        return false;
    }
    syncDirectory(target.parent_path());
    return true;
}

inline bool begin(const std::filesystem::path& directory, const std::string& eventName,
                  std::int64_t acceptedAtNs) {
    std::ostringstream content;
    content << "event=" << eventName << "\naccepted_at_ns=" << acceptedAtNs << "\n";
    return writeAtomically(directory / ".pending", content.str());
}

inline bool fail(const std::filesystem::path& directory, const std::string& reason) {
    std::ostringstream content;
    content << "reason=" << reason << "\nfailed_at_ns="
            << std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count()
            << "\n";
    const bool written = writeAtomically(directory / ".failed", content.str());
    if (written) {
        std::error_code error;
        std::filesystem::remove(directory / ".pending", error);
        std::filesystem::remove(directory / ".complete", error);
    }
    return written;
}

inline bool complete(const std::filesystem::path& directory, std::size_t recordCount) {
    std::ostringstream content;
    content << "records=" << recordCount << "\n";
    if (!writeAtomically(directory / ".complete", content.str())) {
        return false;
    }
    std::error_code error;
    std::filesystem::remove(directory / ".pending", error);
    std::filesystem::remove(directory / ".failed", error);
    syncDirectory(directory);
    return true;
}

struct ReconcileResult {
    std::size_t recoveredFailures{0};
    std::size_t clearedPending{0};
};

inline ReconcileResult reconcile(const std::filesystem::path& root) {
    ReconcileResult result;
    std::error_code iterationError;
    for (std::filesystem::directory_iterator it(root, iterationError), end;
         !iterationError && it != end; it.increment(iterationError)) {
        std::error_code error;
        if (!it->is_directory(error) || error) {
            continue;
        }
        const auto directory = it->path();
        if (!std::filesystem::exists(directory / ".pending", error)) {
            continue;
        }
        if (std::filesystem::exists(directory / ".complete", error) ||
            std::filesystem::exists(directory / ".failed", error)) {
            std::filesystem::remove(directory / ".pending", error);
            ++result.clearedPending;
        } else if (fail(directory, "crash_recovery")) {
            ++result.recoveredFailures;
        }
    }
    return result;
}

} // namespace event_state
