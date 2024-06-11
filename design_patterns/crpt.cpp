#include <iostream>
#include <memory>

// Base class template
template <typename Derived>
struct Base
{
    void interface()
    {
        // static_cast to the derived type
        static_cast<Derived *>(this)->implementation();
    }

    // Default implementation, can be overridden
    // void implementation()
    // {
    //     std::cout << "Default implementation\n";
    // }
};

// Derived class
class Derived : public Base<Derived>
{
public:
    void implementation()
    {
        std::cout << "Derived implementation\n";
    }
};

int main()
{
    Derived d;
    d.interface(); // Calls Derived's implementation
}
