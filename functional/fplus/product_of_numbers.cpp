#include <cstdint>
#include <fplus/fplus.hpp>

int main()
{
    const std::string input = "1,2,3,4,5,6,7,8,9,10";
    // const auto numbers = fplus::split(',', false, input);
    // const auto numbers_ints = fplus::transform(fplus::read_value_unsafe<int>, numbers);
    // const auto result = fplus::product(numbers_ints);
    const auto result = fplus::fwd::apply(input,
                                          fplus::fwd::split(',', false),
                                          fplus::fwd::transform(fplus::read_value_unsafe<int>),
                                          fplus::fwd::product());
    std::cout << result << std::endl;
}