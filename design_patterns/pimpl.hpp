#include <memory>

template <typename T>
class pimpl
{
private:
    std::unique_ptr<T> impl;

public:
    pimpl() : impl(std::make_unique<T>()) {}
    ~pimpl() = default;

    // Delete copy semantics
    pimpl(const pimpl &) = delete;
    pimpl &operator=(const pimpl &) = delete;

    // Allow move semantics
    pimpl(pimpl &&) noexcept = default;
    pimpl &operator=(pimpl &&) noexcept = default;

    template <typename... Args>
    explicit pimpl(Args &&...args)
        : impl(std::make_unique<T>(std::forward<Args>(args)...)) {}

    T *operator->() const noexcept { return impl.get(); }
    T &operator*() const
    {
        assert(impl != nullptr); // or throw std::logic_error
        return *impl;
    }
};
