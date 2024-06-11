#ifndef LOG_HPP
#define LOG_HPP

#include <utility>

template <typename...>
struct log_config; // Forward declaration of a template struct

template <typename... DummyArgs, typename... Args>
auto log(Args &&...args) -> void
{
    log_config<DummyArgs...>::instance().log(std::forward<Args>(args)...);
}

#define LOG(...) log(__VA_ARGS__)

#endif // LOG_HPP
