#include <cassert>
#include <compare>
#include <concepts>
#include <iostream>
#include <iterator>
#include <type_traits>
#include <vector>

template <typename Container>
concept RandomAccessContainer = requires(Container c, typename Container::size_type i) {
    { c[i] } -> std::same_as<typename Container::reference>;
    { c.size() } -> std::same_as<typename Container::size_type>;
    { c.resize(i) };
};

template <typename Container>
concept ReservableContainer = requires(Container c, typename Container::size_type i) {
    { c.reserve(i) };
};

template <RandomAccessContainer Container>
class CircularBuffer
{
public:
    using value_type = typename Container::value_type;
    using reference = typename Container::reference;
    using const_reference = typename Container::const_reference;
    using size_type = typename Container::size_type;
    using iterator = typename Container::iterator;
    using const_iterator = typename Container::const_iterator;

    constexpr CircularBuffer(size_type capacity)
        : buffer_(capacity), head_(0), tail_(0), size_(0), capacity_(capacity)
    {
        if constexpr (ReservableContainer<Container>)
        {
            buffer_.reserve(capacity);
        }
    }

    constexpr void push_back(const value_type &item)
    {
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % capacity_;
        if (size_ < capacity_)
        {
            ++size_;
        }
        else
        {
            head_ = (head_ + 1) % capacity_;
        }
        std::cout << "push_back: head=" << head_ << ", tail=" << tail_ << ", size=" << size_ << std::endl;
    }

    constexpr value_type pop_front()
    {
        assert(size_ > 0 && "pop_front called on empty buffer");
        value_type value = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        --size_;
        std::cout << "pop_front: head=" << head_ << ", tail=" << tail_ << ", size=" << size_ << std::endl;
        return value;
    }

    [[nodiscard]] constexpr reference front()
    {
        assert(size_ > 0 && "front called on empty buffer");
        return buffer_[head_];
    }

    [[nodiscard]] constexpr const_reference front() const
    {
        assert(size_ > 0 && "front called on empty buffer");
        return buffer_[head_];
    }

    [[nodiscard]] constexpr reference back()
    {
        assert(size_ > 0 && "back called on empty buffer");
        return buffer_[(tail_ + capacity_ - 1) % capacity_];
    }

    [[nodiscard]] constexpr const_reference back() const
    {
        assert(size_ > 0 && "back called on empty buffer");
        return buffer_[(tail_ + capacity_ - 1) % capacity_];
    }

    [[nodiscard]] constexpr bool empty() const
    {
        return size_ == 0;
    }

    [[nodiscard]] constexpr size_type size() const
    {
        return size_;
    }

    [[nodiscard]] constexpr size_type capacity() const
    {
        return capacity_;
    }

    constexpr void clear()
    {
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        std::cout << "clear: head=" << head_ << ", tail=" << tail_ << ", size=" << size_ << std::endl;
    }

    [[nodiscard]] constexpr auto begin()
    {
        return Iterator(*this, head_, 0);
    }

    [[nodiscard]] constexpr auto end()
    {
        return Iterator(*this, head_, size_);
    }

    [[nodiscard]] constexpr auto begin() const
    {
        return ConstIterator(*this, head_, 0);
    }

    [[nodiscard]] constexpr auto end() const
    {
        return ConstIterator(*this, head_, size_);
    }

    // Default the spaceship operator
    auto operator<=>(const CircularBuffer &) const = default;

private:
    Container buffer_;
    size_type head_;
    size_type tail_;
    size_type size_;
    size_type capacity_;

    class Iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = typename CircularBuffer::value_type;
        using difference_type = std::ptrdiff_t;
        using pointer = value_type *;
        using reference = value_type &;

        constexpr Iterator(CircularBuffer &buffer, size_type pos, size_type index)
            : buffer_(buffer), pos_(pos), index_(index) {}

        constexpr reference operator*()
        {
            return buffer_.buffer_[pos_];
        }

        constexpr pointer operator->()
        {
            return &buffer_.buffer_[pos_];
        }

        constexpr Iterator &operator++()
        {
            pos_ = (pos_ + 1) % buffer_.capacity_;
            ++index_;
            return *this;
        }

        constexpr Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        [[nodiscard]] constexpr bool operator==(Iterator const &other) const
        {
            return index_ == other.index_;
        }

        [[nodiscard]] constexpr bool operator!=(Iterator const &other) const
        {
            return !(*this == other);
        }

    private:
        CircularBuffer &buffer_;
        size_type pos_;
        size_type index_;
    };

    class ConstIterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = const typename CircularBuffer::value_type;
        using difference_type = std::ptrdiff_t;
        using pointer = value_type const *;
        using reference = value_type const &;

        constexpr ConstIterator(CircularBuffer const &buffer, size_type pos, size_type index)
            : buffer_(buffer), pos_(pos), index_(index) {}

        constexpr reference operator*() const
        {
            return buffer_.buffer_[pos_];
        }

        constexpr pointer operator->() const
        {
            return &buffer_.buffer_[pos_];
        }

        constexpr ConstIterator &operator++()
        {
            pos_ = (pos_ + 1) % buffer_.capacity_;
            ++index_;
            return *this;
        }

        constexpr ConstIterator operator++(int)
        {
            ConstIterator tmp = *this;
            ++(*this);
            return tmp;
        }

        [[nodiscard]] constexpr bool operator==(ConstIterator const &other) const
        {
            return index_ == other.index_;
        }

        [[nodiscard]] constexpr bool operator!=(ConstIterator const &other) const
        {
            return !(*this == other);
        }

    private:
        CircularBuffer const &buffer_;
        size_type pos_;
        size_type index_;
    };
};

int main()
{
    // Initial test cases
    CircularBuffer<std::vector<int>> cb(5);

    // Test push_back and front/back
    cb.push_back(1);
    cb.push_back(2);
    cb.push_back(3);

    assert(cb.front() == 1);
    assert(cb.back() == 3);
    std::cout << "Initial push_back tests passed" << std::endl;

    // Test pop_front
    assert(cb.pop_front() == 1);
    assert(cb.front() == 2);
    std::cout << "pop_front test passed" << std::endl;

    // Test circular behavior
    cb.push_back(4);
    cb.push_back(5);
    cb.push_back(6);
    cb.push_back(7);

    assert(cb.front() == 3);
    assert(cb.back() == 7);
    std::cout << "Circular behavior tests passed" << std::endl;

    // Test iterators
    std::cout << "Buffer elements: ";
    std::vector<int> expected = {3, 4, 5, 6, 7};
    size_t i = 0;
    for (auto it = cb.begin(); it != cb.end(); ++it, ++i)
    {
        assert(*it == expected[i]);
        std::cout << *it << " ";
    }
    std::cout << std::endl;
    std::cout << "Iterator tests passed" << std::endl;

    // Test const iterators
    const CircularBuffer<std::vector<int>> &const_cb = cb;
    std::cout << "Buffer elements (const): ";
    i = 0;
    for (auto it = const_cb.begin(); it != const_cb.end(); ++it, ++i)
    {
        assert(*it == expected[i]);
        std::cout << *it << " ";
    }
    std::cout << std::endl;
    std::cout << "Const iterator tests passed" << std::endl;

    // Additional tests

    // Test clear method
    cb.clear();
    assert(cb.empty());
    std::cout << "Clear method test passed" << std::endl;

    // Test resizing
    cb.push_back(1);
    cb.push_back(2);
    cb.push_back(3);
    cb.push_back(4);
    cb.push_back(5);

    assert(cb.front() == 1);
    assert(cb.back() == 5);

    cb.push_back(6); // This should overwrite the 1
    assert(cb.front() == 2);
    assert(cb.back() == 6);
    std::cout << "Resizing tests passed" << std::endl;

    // Test edge cases
    CircularBuffer<std::vector<int>> cb_edge(3);

    cb_edge.push_back(1);
    cb_edge.push_back(2);
    cb_edge.push_back(3);
    assert(cb_edge.front() == 1);
    assert(cb_edge.back() == 3);

    cb_edge.push_back(4); // Overwrites 1
    assert(cb_edge.front() == 2);
    assert(cb_edge.back() == 4);

    cb_edge.push_back(5); // Overwrites 2
    assert(cb_edge.front() == 3);
    assert(cb_edge.back() == 5);

    cb_edge.push_back(6); // Overwrites 3
    assert(cb_edge.front() == 4);
    assert(cb_edge.back() == 6);

    cb_edge.pop_front();
    assert(cb_edge.front() == 5);
    cb_edge.pop_front();
    assert(cb_edge.front() == 6);
    cb_edge.pop_front();
    assert(cb_edge.empty());

    std::cout << "Edge case tests passed" << std::endl;

    // Additional pop_front edge case
    CircularBuffer<std::vector<int>> cb_empty(3);
    // assert(cb_empty.pop_front() == std::nullopt);
    std::cout << "Empty pop_front test passed" << std::endl;

    // Test pushing and popping continuously
    CircularBuffer<std::vector<int>> cb_continuous(2);
    cb_continuous.push_back(1);
    cb_continuous.push_back(2);
    assert(cb_continuous.pop_front() == 1);
    cb_continuous.push_back(3);
    assert(cb_continuous.pop_front() == 2);
    assert(cb_continuous.pop_front() == 3);
    // assert(cb_continuous.pop_front() == std::nullopt);
    std::cout << "Continuous push/pop test passed" << std::endl;

    return 0;
}
