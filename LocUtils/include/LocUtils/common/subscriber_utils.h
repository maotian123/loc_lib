//
// Shared subscriber utility: thread-safe buffer drain used by all subscribers.
//
#pragma once

#include <deque>
#include <mutex>

namespace LocUtils {

template <typename T>
void DrainBuffer(std::mutex& mutex, std::deque<T>& src, std::deque<T>& dst)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (!src.empty()) {
        dst.insert(dst.end(), src.begin(), src.end());
        src.clear();
    }
}

}  // namespace LocUtils
