#pragma once

#include <array>
#include <cassert>
#include <iterator>
#include <span>
#include <type_traits>
#include <vector>

namespace cyclic
{

    // Iterator boilerplate using std::iterator
    template <typename ContainerType>
    class CyclicIterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = typename ContainerType::value_type;
        using difference_type = typename ContainerType::difference_type;
        using pointer = typename std::conditional_t<std::is_const_v<ContainerType>, typename ContainerType::const_pointer, typename ContainerType::pointer>;
        using reference = typename std::conditional_t<std::is_const_v<ContainerType>, typename ContainerType::const_reference, typename ContainerType::reference>;

        CyclicIterator(ContainerType &container, typename ContainerType::size_type start, typename ContainerType::size_type index)
            : container_(container), start_(start), index_(index) {}

        reference operator*() const
        {
            return container_[(start_ + index_) % container_.size()];
        }

        pointer operator->() const
        {
            return &container_[(start_ + index_) % container_.size()];
        }

        CyclicIterator &operator++()
        {
            ++index_;
            return *this;
        }

        CyclicIterator operator++(int)
        {
            CyclicIterator tmp = *this;
            ++index_;
            return tmp;
        }

        friend bool operator==(const CyclicIterator &a, const CyclicIterator &b)
        {
            return a.index_ == b.index_ && &a.container_ == &b.container_;
        }

        friend bool operator!=(const CyclicIterator &a, const CyclicIterator &b)
        {
            return !(a == b);
        }

        auto operator<=>(const CyclicIterator &other) const = default;

    private:
        ContainerType &container_;
        typename ContainerType::size_type start_;
        typename ContainerType::size_type index_;
    };

    // Template class definition for standard containers
    template <typename ContainerType>
    class CyclicView
    {
    public:
        using size_type = typename ContainerType::size_type;
        using iterator = CyclicIterator<ContainerType>;
        using const_iterator = CyclicIterator<const ContainerType>;

        constexpr CyclicView(ContainerType &container, size_type capacity, size_type start = 0)
            : container_(container), capacity_(capacity), start_(start), size_(container.size()), push_index_(start)
        {
            assert(((start == 0 && size_ == 0) || (start < size_)) && "Invalid start index for CyclicView");
        }
        template <typename T, size_t N>
        constexpr CyclicView(std::array<typename ContainerType::value_type, N> &array, size_type size = 0, size_type start = 0)
            : container_(array), capacity_(N), start_(start), size_(size), push_index_(start)
        {
            assert(((start == 0 && size_ == 0) || (start < size_)) && "Invalid start index for CyclicView");
        }

        [[nodiscard]] constexpr const auto &operator[](size_type index) const
        {
            return container_[(start_ + index) % capacity_];
        }

        [[nodiscard]] constexpr size_type size() const noexcept
        {
            return container_.size();
        }

        template<typename T, size_t N>
        [[nodiscard]] constexpr size_type size(const std::array<T, N>&) const noexcept
        {
            return size_;
        }

        iterator begin()
        {
            return iterator(container_, start_, 0);
        }

        iterator end()
        {
            return iterator(container_, start_, size_);
        }

        const_iterator begin() const
        {
            return const_iterator(container_, start_, 0);
        }

        const_iterator end() const
        {
            return const_iterator(container_, start_, size_);
        }

        void push(const typename ContainerType::value_type &value)
        {
            if (this->size() < capacity_)
            {
                insert_element(container_, this->size() + push_index_, value);
            }
            else
            {
                container_[push_index_] = value;
                start_ = (start_ + 1) % this->size();
            }
            push_index_ = (push_index_ + 1) % this->size();
        }

    private:
        template <typename C>
        void insert_element(C &container, size_type index, const typename C::value_type &value)
        {
            container.emplace(container.begin() + index, value);
        }

        template <typename T, size_t N>
        void insert_element(std::array<T, N> &container, size_type index, const T &value)
        {
            container[index] = value;
            ++size_;
        }

        // bool invariant() const
        // {
        //     return start_ < capacity_ && size_ <= container_ && push_index_ < container_.size() && container_.size() <= capacity_;
        // }

        ContainerType &container_;
        size_type capacity_;
        size_type start_;
        size_type size_;
        size_type push_index_;
    };

    // Specialization for C-style pointers
    template <typename T>
    class CyclicView<T *>
    {
    public:
        using size_type = size_t;
        using value_type = T;
        using iterator = CyclicIterator<std::span<T>>;
        using const_iterator = CyclicIterator<const std::span<T>>;

        constexpr CyclicView(T *array, size_type capacity, size_type size = 0, size_type start = 0)
            : container_(array, capacity), capacity_(capacity), start_(start), size_(size), push_index_(size % capacity)
        {
            assert(array != nullptr && "Invalid array pointer for CyclicView");
            assert(start < capacity && "Invalid start index for CyclicView");
            assert(size <= capacity && "Size exceeds capacity");
        }

        [[nodiscard]] constexpr const auto &operator[](size_type index) const
        {
            return container_[(start_ + index) % capacity_];
        }

        [[nodiscard]] constexpr size_type size() const noexcept
        {
            return size_;
        }

        iterator begin()
        {
            return iterator(container_, start_, 0);
        }

        iterator end()
        {
            return iterator(container_, start_, size_);
        }

        const_iterator begin() const
        {
            return const_iterator(container_, start_, 0);
        }

        const_iterator end() const
        {
            return const_iterator(container_, start_, size_);
        }

        void push(const value_type &value)
        {
            assert(size_ <= capacity_ && "Cannot push beyond capacity for C-style array");
            container_[push_index_] = value;
            push_index_ = (push_index_ + 1) % size_;
            if (size_ < capacity_)
            {
                ++size_;
            }
            else
            {
                start_ = (start_ + 1) % size_;
            }
        }

    private:
        std::span<T> container_;
        size_type capacity_;
        size_type start_;
        size_type size_;
        size_type push_index_;
    };

    // Deduction guide
    template <typename ContainerType>
    CyclicView(ContainerType &, typename ContainerType::size_type, typename ContainerType::size_type) -> CyclicView<ContainerType>;

    template <typename ContainerType>
    CyclicView(ContainerType &) -> CyclicView<ContainerType>;

    template <typename T>
    CyclicView(T *, size_t, size_t) -> CyclicView<T *>;

} // namespace cyclic
