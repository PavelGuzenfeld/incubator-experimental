#include <cassert>
#include <iostream>
#include <memory>

// Base template class for pImpl using CRTP
template <typename Derived, typename Impl>
class PimplBase
{
protected:
    std::unique_ptr<Impl> impl;

public:
    PimplBase() : impl(std::make_unique<Impl>()) {}
    template <typename... Args>
    explicit PimplBase(Args &&...args) : impl(std::make_unique<Impl>(std::forward<Args>(args)...)) {}

    ~PimplBase() = default;

    PimplBase(const PimplBase &) = delete;
    PimplBase &operator=(const PimplBase &) = delete;

    PimplBase(PimplBase &&) noexcept = default;
    PimplBase &operator=(PimplBase &&) noexcept = default;

    Impl *operator->() const noexcept { return impl.get(); }
    Impl &operator*() const
    {
        assert(impl != nullptr);
        return *impl;
    }
};

// Implementation details for Widget
class WidgetImpl
{
private:
    int data;

public:
    explicit WidgetImpl(int data) : data(data) {}

    void draw() const
    {
        std::cout << "Drawing a widget with data: " << data << std::endl;
    }

    int getData() const
    {
        return data;
    }
};

// Widget class using CRTP with pImpl
class Widget : public PimplBase<Widget, WidgetImpl>
{
public:
    using PimplBase<Widget, WidgetImpl>::PimplBase; // Inherit all constructors from PimplBase

    void draw() const
    {
        impl->draw();
    }

    int getData() const
    {
        return impl->getData();
    }
};

// Main function to demonstrate usage
int main()
{
    Widget w(42);                                             // Create a Widget with some internal data
    w.draw();                                                 // Output: Drawing a widget with data: 42
    std::cout << "Widget data: " << w.getData() << std::endl; // Output: Widget data: 42

    return 0;
}
