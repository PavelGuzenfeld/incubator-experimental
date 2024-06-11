#include <cstdint>
#include <fplus/fplus.hpp>

int main()
{
    // Exercise 1:
    //     Square every inner value of xss, so
    //     {{0,1,2}, {3,4,5}}
    //     turns into
    //     {{0,1,4}, {9,16,25}}
    std::vector<std::vector<int>> xss =
        {{0, 1, 2}, {3, 4, 5}};

    auto result = fplus::transform(fplus::fwd::transform(fplus::square<int>), xss);
    std::cout << fplus::show(result) << std::endl;
}