#include <iostream>
#include <utility>

// Base class for PMPL with CRTP. The derived class provides its implementation.
template <typename Derived>
class CompileFirewallCRTP
{
public:
    constexpr auto do_thing() -> void
    {
        // Delegate to the derived class (static polymorphism)
        static_cast<Derived *>(this)->do_thing();
    }
};

// Policy 1: One concrete implementation
class Policy1 : public CompileFirewallCRTP<Policy1>
{
public:
    // Example of what the firewall protects: isolated implementation
    constexpr auto do_thing() -> void
    {
        std::cout << "Policy1: Doing something in Policy1!\n";
    }
};

// Policy 2: Another concrete implementation
class Policy2 : public CompileFirewallCRTP<Policy2>
{
public:
    // A completely different implementation hidden behind the firewall
    constexpr auto do_thing() -> void
    {
        std::cout << "Policy2: Doing something else in Policy2!\n";
    }
};

// Function to create the firewall; returns the correct policy instance.
template <typename Policy>
constexpr auto create_firewall() -> Policy
{
    return Policy();
}

int main()
{
    // Create firewall with Policy1
    auto firewall1 = create_firewall<Policy1>();
    firewall1.do_thing(); // Output: Policy1: Doing something in Policy1!

    // Create firewall with Policy2
    auto firewall2 = create_firewall<Policy2>();
    firewall2.do_thing(); // Output: Policy2: Doing something else in Policy2!

    return 0;
}
