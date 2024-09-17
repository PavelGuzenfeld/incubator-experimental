#include <iostream>
#include <optional>
#include <string>
#include <utility>

template <typename Ret>
auto validated = []<typename... Vals>(Vals &&...vals)
{
    return [... validators = std::forward<Vals>(vals)]<typename... Args>(Args &&...args) -> std::optional<Ret>
    {
        if ((validators(args...) && ...))
        {
            return std::optional<Ret>{Ret{std::forward<Args>(args)...}};
        }
        else
        {
            return std::nullopt;
        }
    };
};

// Example validators
auto length_validator = [](const std::string &str)
{ return str.size() >= 8; };
auto special_char_validator = [](const std::string &str)
{ return str.find_first_of("!@#$%^&*") != std::string::npos; };

int main()
{
    // Create a validator with multiple rules
    auto pswd_validator = validated<std::string>(length_validator, special_char_validator);

    // Example usage
    std::cout << pswd_validator("abcde!gh").value_or("Password error") << '\n'; // Valid case
    std::cout << pswd_validator("short").value_or("Password error") << '\n';    // Invalid case
}
