#include <iostream>
#include <string>
#include <tuple>

// Include the function template you've so generously provided
namespace util
{
    template <std::size_t I = 0, typename Func, typename... Ts>
    constexpr void apply(Func func, const std::tuple<Ts...> &tuple)
    {
        if constexpr (I < sizeof...(Ts))
        {
            func(std::get<I>(tuple));  // Apply the function to the current element
            apply<I + 1>(func, tuple); // Move to the next element recursively
        }
    }
}

int main()
{
    // Create a tuple with different types. Because why make anything simple?
    auto my_tuple = std::make_tuple(1, 3.14, "Hello", std::string("World"));

    // Define a function to apply. Here's one that prints whatever it gets.
    auto print_func = [](const auto &elem)
    {
        std::cout << elem << '\n';
    };

    // Call the apply function with the tuple and the print function.
    util::apply(print_func, my_tuple);

    return 0;
}
