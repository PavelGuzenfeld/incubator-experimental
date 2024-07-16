#pragma once

#include <vector>
#include <array>
#include <iostream>
#include <stdexcept>
#include <iterator>
#include <type_traits>

namespace cyclic {

// Iterator boilerplate using std::iterator
template<typename ContainerType>
class CyclicIterator {
public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = typename ContainerType::value_type;
    using difference_type = typename ContainerType::difference_type;
    using pointer = typename ContainerType::pointer;
    using reference = typename ContainerType::reference;

    CyclicIterator(ContainerType& container, typename ContainerType::size_type start, typename ContainerType::size_type index)
        : container_(container), start_(start), index_(index) {}

    reference operator*() {
        return container_[(start_ + index_) % container_.size()];
    }

    pointer operator->() {
        return &container_[(start_ + index_) % container_.size()];
    }

    CyclicIterator& operator++() {
        ++index_;
        return *this;
    }

    CyclicIterator operator++(int) {
        CyclicIterator tmp = *this;
        ++index_;
        return tmp;
    }

    friend bool operator==(const CyclicIterator& a, const CyclicIterator& b) {
        return a.index_ == b.index_ && &a.container_ == &b.container_;
    }

    friend bool operator!=(const CyclicIterator& a, const CyclicIterator& b) {
        return !(a == b);
    }

    auto operator<=>(const CyclicIterator& other) const = default;

private:
    ContainerType& container_;
    typename ContainerType::size_type start_;
    typename ContainerType::size_type index_;
};

// Template class definition
template <typename ContainerType>
class CyclicView {
public:
    using size_type = typename ContainerType::size_type;
    using iterator = CyclicIterator<ContainerType>;

    constexpr CyclicView(ContainerType& container, size_type start, size_type size)
        : container_(container), start_(start), size_(size), push_index_(0) {
        if (start_ >= container_.size() || size_ > container_.size()) {
            throw std::out_of_range("Invalid start or size for CyclicView");
        }
    }

    [[nodiscard]] constexpr const auto& operator[](size_type index) const {
        return container_[(start_ + index) % container_.size()];
    }

    [[nodiscard]] constexpr size_type size() const noexcept {
        return size_;
    }

    iterator begin() {
        return iterator(container_, start_, 0);
    }

    iterator end() {
        return iterator(container_, start_, size_);
    }

    void push(const typename ContainerType::value_type& value) {
        container_[(start_ + push_index_) % container_.size()] = value;
        push_index_ = (push_index_ + 1) % container_.size();
        if (size_ < container_.size()) {
            ++size_;
        }
    }

private:
    ContainerType& container_;
    size_type start_;
    size_type size_;
    size_type push_index_;
};

// Helper function to create CyclicView from a C-style array
template <typename T, size_t N>
[[nodiscard]] constexpr auto make_cyclic_view(T (&array)[N]) {
    // Here we use a static vector to ensure its lifetime extends beyond the function scope
    static std::vector<T> container(array, array + N);
    return CyclicView<std::vector<T>>(container, 0, N);
}

} // namespace cyclic
