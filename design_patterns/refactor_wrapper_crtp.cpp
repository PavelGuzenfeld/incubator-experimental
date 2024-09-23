#include <iostream>
#include <utility>

// CRTP base class template for refactoring
template <typename Derived, typename TOld>
class RefactorWrapperCRTP
{
public:
    // Template constructor for perfect forwarding
    template <typename T>
    explicit RefactorWrapperCRTP(T &&old_obj)
        : old_obj_(std::forward<T>(old_obj)) {}

    // Old implementation access.
    [[nodiscard]] constexpr auto &old() { return old_obj_; }

    // CRTP magic: call the derived class's method via static polymorphism.
    constexpr auto invoke_new_method() -> void
    {
        static_cast<Derived *>(this)->do_new_thing();
    }

private:
    TOld old_obj_; // Old class you're refactoring from.
};

// Old class you need to refactor.
class OldClass
{
public:
    constexpr auto do_old_thing() const -> void
    {
        std::cout << "OldClass: doing the old thing...\n";
    }

    constexpr auto old_method_with_params(int x) const -> void
    {
        std::cout << "OldClass: old method with params: " << x << "\n";
    }
};

// Your new fancy class using CRTP to avoid virtual functions.
class NewPolicy : public RefactorWrapperCRTP<NewPolicy, OldClass>
{
public:
    // Pass through the old object using perfect forwarding
    template <typename T>
    explicit NewPolicy(T &&old_obj)
        : RefactorWrapperCRTP<NewPolicy, OldClass>(std::forward<T>(old_obj)) {}

    // Here's where we define the new behavior.
    constexpr auto do_new_thing() const -> void
    {
        std::cout << "NewPolicy: doing something new with CRTP!\n";
    }

    constexpr auto new_method_with_params(int x, const std::string &str) const -> void
    {
        std::cout << "NewPolicy: new method with params: " << x << ", " << str << "\n";
    }
};

// Utility function to create the CRTP-based wrapper.
template <typename TOld>
[[nodiscard]] auto create_refactor_crtp(TOld &&old_class)
{
    return NewPolicy(std::forward<TOld>(old_class));
}

int main()
{
    OldClass old_obj{};

    // Create the CRTP-based refactor wrapper
    auto crtp_refactored_obj = create_refactor_crtp(old_obj);

    // Call methods directly on the old object.
    crtp_refactored_obj.old().do_old_thing();
    crtp_refactored_obj.old().old_method_with_params(42);

    // Call new methods through CRTP-based derived class.
    crtp_refactored_obj.invoke_new_method();                         // Output: NewPolicy: doing something new with CRTP!
    crtp_refactored_obj.new_method_with_params(42, "Hello, world!"); // Output: NewPolicy: new method with params...

    return 0;
}
