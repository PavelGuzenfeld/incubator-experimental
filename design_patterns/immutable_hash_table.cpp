#include <algorithm>
#include <array>
#include <concepts> // For concept definition
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
// Concept to check if a type is a hash function
template <typename T, typename Key>
concept HashFunction = requires(T t, Key k) {
    { t(k) } -> std::convertible_to<std::size_t>;
};
// Primary template for const_hash, left undefined to require specializations
template <typename T>
struct const_hash;
// Specialization of const_hash for std::string_view
template <>
struct const_hash<std::string_view>
{
    constexpr std::size_t operator()(std::string_view input) const
    {
        std::size_t hash = 0;
        for (char c : input)
        {
            hash = hash * 31 + static_cast<std::size_t>(c);
        }
        return hash;
    }
};
// Immutable hash table
template <typename K, typename V, std::size_t N, HashFunction<K> Hash = const_hash<K>>
class ImmutableHashTable
{
public:
    // Type aliases for STL compatibility
    using value_type = std::pair<K, V>;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type &;
    using const_reference = const value_type &;
    using iterator = typename std::array<value_type, N>::iterator;
    using const_iterator = typename std::array<value_type, N>::const_iterator;
    constexpr ImmutableHashTable(const std::pair<K, V> (&arr)[N], Hash hash_fn = Hash{})
        : table{}, hash_function(hash_fn)
    {
        for (std::size_t i = 0; i < N; ++i)
        {
            table[i] = arr[i];
        }
    }
    // Default constructor
    constexpr ImmutableHashTable() : table{}, hash_function(Hash{}) {}
    // Copy constructor
    constexpr ImmutableHashTable(const ImmutableHashTable &other)
        : table(other.table), hash_function(other.hash_function) {}
    // Move constructor
    constexpr ImmutableHashTable(ImmutableHashTable &&other) noexcept
        : table(std::move(other.table)), hash_function(std::move(other.hash_function)) {}
    // Copy assignment operator
    ImmutableHashTable &operator=(const ImmutableHashTable &other)
    {
        if (this != &other)
        {
            table = other.table;
            hash_function = other.hash_function;
        }
        return *this;
    }
    // Move assignment operator
    ImmutableHashTable &operator=(ImmutableHashTable &&other) noexcept
    {
        table = std::move(other.table);
        hash_function = std::move(other.hash_function);
        return *this;
    }
    // Access methods
    constexpr const_reference at(size_type index) const
    {
        if (index >= N)
        {
            throw std::out_of_range("Index out of range");
        }
        return table[index];
    }

    constexpr std::optional<V> find(const K &key) const
    {
        std::size_t hash = hash_function(key);
        for (const auto &[pair_key, pair_value] : table)
        {
            if (hash_function(pair_key) == hash && std::equal_to<K>{}(pair_key, key))
            {
                return pair_value;
            }
        }
        return std::nullopt;
    }
    constexpr const_reference front() const { return table.front(); }
    constexpr const_reference back() const { return table.back(); }
    constexpr std::size_t size() const
    {
        return N;
    }
    constexpr iterator begin() const
    {
        return iterator(table.begin());
    }
    constexpr iterator end() const
    {
        return iterator(table.end());
    }

private:
    std::array<std::pair<K, V>, N> table;
    Hash hash_function;
};
// Example usage
constexpr std::pair<std::string_view, int> kv_pairs[] = {
    {"key1", 1}, {"key2", 2}};
int main()
{
    constexpr ImmutableHashTable<std::string_view, int, 2> myHashTable(kv_pairs);
    static_assert(myHashTable.find("key1").has_value() && myHashTable.find("key1").value() == 1);
    static_assert(myHashTable.find("key2").has_value() && myHashTable.find("key2").value() == 2);
    static_assert(!myHashTable.find("key3").has_value());
    // Example of using STL algorithms with the hash table
    auto it = std::find_if(myHashTable.begin(), myHashTable.end(), [](const auto &pair)
                           { return pair.first == "key1"; });
    if (it != myHashTable.end())
    {
        // Found key1
    }
    return 0;
}