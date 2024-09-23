#include <iostream>
#include <memory>
#include <utility>

// Abstract base class providing the interface
class IThingDoer
{
public:
    virtual ~IThingDoer() = default;

    // Abstract method that derived classes must implement
    virtual void do_thing() const = 0;
};

// PMPL CRTP Base class that uses policies for implementations
template <typename Derived, typename Policy>
class CompileFirewallCRTP : public IThingDoer
{
public:
    // Interface implementation delegating to the CRTP derived class
    void do_thing() const override
    {
        static_cast<const Derived *>(this)->do_thing();
    }
};

// Policy 1: Defines a specific implementation detail
struct Policy1
{
    constexpr void do_thing() const
    {
        std::cout << "Policy1: Doing something in Policy1!\n";
    }
};

// Policy 2: Defines another implementation detail
struct Policy2
{
    constexpr void do_thing() const
    {
        std::cout << "Policy2: Doing something different in Policy2!\n";
    }
};

// Concrete class 1 that inherits from the CRTP and uses Policy1
class ThingDoerImpl1 : public CompileFirewallCRTP<ThingDoerImpl1, Policy1>
{
public:
    // Constructor using perfect forwarding (if needed for more complex cases)
    explicit ThingDoerImpl1() = default;

    // Using the policy's behavior for implementation
    constexpr void do_thing() const
    {
        Policy1{}.do_thing();
    }
};

// Concrete class 2 that inherits from the CRTP and uses Policy2
class ThingDoerImpl2 : public CompileFirewallCRTP<ThingDoerImpl2, Policy2>
{
public:
    // Constructor using perfect forwarding (if needed for more complex cases)
    explicit ThingDoerImpl2() = default;

    // Using the policy's behavior for implementation
    constexpr void do_thing() const
    {
        Policy2{}.do_thing();
    }
};

// Utility function to create the appropriate implementation
template <typename Impl>
std::unique_ptr<IThingDoer> create_firewall()
{
    return std::make_unique<Impl>();
}

int main()
{
    // Create firewall with ThingDoerImpl1 (using Policy1)
    auto firewall1 = create_firewall<ThingDoerImpl1>();
    firewall1->do_thing(); // Output: Policy1: Doing something in Policy1!

    // Create firewall with ThingDoerImpl2 (using Policy2)
    auto firewall2 = create_firewall<ThingDoerImpl2>();
    firewall2->do_thing(); // Output: Policy2: Doing something different in Policy2!

    return 0;
}
