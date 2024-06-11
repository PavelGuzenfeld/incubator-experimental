#include "log_impl.hpp"

// Implementation that logs to stdout
struct my_logger
{
    template <typename... Args>
    auto log(Args &&...args) const -> void
    {
        std::cout << "(my_logger) ";
        (std::cout << ... << args) << std::endl;
    }
};

// Specializations for my_logger and test_logger
template <>
struct log_config<>
{
    static my_logger &instance()
    {
        static my_logger logger;
        return logger;
    }
};

int main()
{
    // Using the null logger (does nothing)
    LOG("This will not be logged", 123, 456);



    // Switching to my_logger (no need for template specialization in main)
    LOG("This will be logged to stdout: ", 42, " and ", 1729);

    // If you need to switch to test_logger, you would define another specialization in log_impl.hpp

    return 0;
}
