#include <iterator>
#include <string_view>
#include <vector>
int main(int const argc, char const *const *const argv)
{
    // argc is the number of command-line arguments.
    // argv is an array of pointers to the command-line arguments, where:
    // argv itself is const: you can't change the pointer to the array of arguments.
    // argv[0] is a const pointer to const char: you can't change the pointers to the individual arguments.
    // argv[0][0] is a const char: you can't change the characters in the arguments.
    // you can't do any of these:
    // argv = nullptr;                 // error: can't modify argv
    // argv[0] = "another string";     // error: can't modify argv[i]
    // argv[0][0] = 'X';               // error: can't modify argv[i][j]
    // auto deduces the type of args, which is std::vector<std::string_view>
    // std::vector<std::string_view> is a vector of string views.
    // std::string_view is a non-owning reference to a string, which means it doesn't copy the string.
    // argv is the start of the range.
    // std::next(argv, static_cast<std::ptrdiff_t>(argc)) is the end of the range, obtained by advancing argv by argc positions.
    // static_cast<std::ptrdiff_t>(argc) casts argc to std::ptrdiff_t, which is the correct type for pointer arithmetic.
    // this constructs a vector of string views, each pointing to one of the command-line arguments.
    auto args = std::vector<std::string_view>(argv, std::next(argv, static_cast<std::ptrdiff_t>(argc)));
    return 0;
}