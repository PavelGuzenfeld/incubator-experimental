#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>

// Memoization template
template <typename Result, typename... Args>
std::function<Result(Args...)> memoize(std::function<Result(Args...)> func)
{
    auto cache = std::make_shared<std::map<std::tuple<Args...>, Result>>();
    return [func, cache](Args... args) mutable -> Result
    {
        const auto args_tuple = std::make_tuple(args...);
        auto &ref_cache = *cache;
        auto it = ref_cache.find(args_tuple);
        if (it != ref_cache.end())
        {
            return it->second;
        }
        else
        {
            Result result = func(args...);
            ref_cache[args_tuple] = result;
            return result;
        }
    };
}

// Example function to memoize
int64_t fib(int64_t n)
{
    return n < 2 ? n : fib(n - 1) + fib(n - 2);
}

int main()
{
    auto memoized_fib = memoize<int64_t, int64_t>(std::function<int64_t(int64_t)>(fib));

    std::cout << memoized_fib(45) << std::endl;
    std::cout << memoized_fib(45) << std::endl;
    std::cout << memoized_fib(45) << std::endl;

    return 0;
}
