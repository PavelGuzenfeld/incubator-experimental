#include <cassert>
#include <iostream>
#include <memory>

// Definition of the pimpl template class
template <typename T>
class pimpl
{
private:
    std::unique_ptr<T> impl;

public:
    pimpl() : impl(std::make_unique<T>()) {}
    ~pimpl() = default;

    pimpl(const pimpl &) = delete;
    pimpl &operator=(const pimpl &) = delete;

    pimpl(pimpl &&) noexcept = default;
    pimpl &operator=(pimpl &&) noexcept = default;

    template <typename... Args>
    explicit pimpl(Args &&...args) : impl(std::make_unique<T>(std::forward<Args>(args)...)) {}

    T *operator->() const noexcept { return impl.get(); }
    T &operator*() const
    {
        assert(impl != nullptr);
        return *impl;
    }
};

// Forward declaration of the implementation class
class WidgetImpl;

// Full definition of WidgetImpl
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

// Widget class using pimpl with WidgetImpl
class Widget
{
    pimpl<WidgetImpl> impl;

public:
    explicit Widget(int data) : impl(data) {}
    ~Widget() = default;

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
    Widget w(42);
    w.draw();                                                 // Output: Drawing a widget with data: 42
    std::cout << "Widget data: " << w.getData() << std::endl; // Output: Widget data: 42

    return 0;
}
