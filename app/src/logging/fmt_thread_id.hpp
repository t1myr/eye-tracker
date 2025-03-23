#ifndef _FMT_THREAD_ID_HPP_
#define _FMT_THREAD_ID_HPP_

#include <fmt/format.h>
#include <thread>

template <>
struct fmt::formatter<std::thread::id> : fmt::formatter<std::string_view> {
    auto format(const std::thread::id& id, fmt::format_context& ctx) const {
        std::stringstream ss;
        ss << id;
        return fmt::formatter<std::string_view>::format(ss.str(), ctx);
    }
};

#endif