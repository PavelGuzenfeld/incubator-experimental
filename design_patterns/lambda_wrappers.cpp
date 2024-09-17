#include <type_traits>

template <typename F>
constexpr auto make_lambda(F &&f)
{
    return [f = std::forward<F>(f)]<typename... Args>(Args &&...args)
    {
        if constexpr (std::is_void_v<decltype(f(std::forward<Args>(args)...))>)
        {
            // If the return type is void
            f(std::forward<Args>(args)...);
        }
        else
        {
            // Directly call the function without forwarding it
            return f(std::forward<Args>(args)...);
        }
    };
}

// Task creator template
template <typename F, typename... Args>
auto make_task(F &&f, Args &&...args)
{
    // Capture the function and arguments by forwarding only the arguments
    return [f = std::forward<F>(f), ... args = std::forward<Args>(args)]() mutable
    {
        // Directly call the function without forwarding it
        return f(std::forward<Args>(args)...);
    };
}