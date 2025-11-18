// Bloom Filter Implementation
// Compiler Explorer: Use C++17, add flag: -lfmt
// Space-efficient probabilistic data structure for set membership testing
// Key properties: Fast lookups, no false negatives, possible false positives

#include <cmath>
#include <cstdint>
#include <fmt/core.h>
#include <functional>
#include <string>
#include <vector>

using BitIndex = uint32_t;
using HashValue = uint64_t;
using NumHashes = uint8_t;

class BloomFilter
{
private:
    std::vector<bool> bit_array;  // Bit array for storing hashed values
    BitIndex num_bits;            // Size of bit array
    NumHashes num_hash_functions; // Number of hash functions to use
    size_t element_count;         // Number of elements inserted

    // Generate k different hash values using double hashing technique
    // h_i(x) = (hash1(x) + i * hash2(x)) mod m
    std::vector<BitIndex> getHashIndices(const std::string &item) const
    {
        std::vector<BitIndex> indices;

        // Two base hash functions
        std::hash<std::string> hasher;
        HashValue hash1 = hasher(item);
        HashValue hash2 = hasher(item + "salt"); // Add salt for different hash

        // Generate k hash values using double hashing
        for (NumHashes i = 0; i < num_hash_functions; i++)
        {
            HashValue combined = hash1 + i * hash2;
            BitIndex index = static_cast<BitIndex>(combined % num_bits);
            indices.push_back(index);
        }

        return indices;
    }

public:
    // Constructor: Calculate optimal size and hash count
    // n = expected number of elements
    // fpr = desired false positive rate (e.g., 0.01 = 1%)
    BloomFilter(size_t expected_elements, double false_positive_rate)
        : element_count(0)
    {

        // Optimal bit array size: m = -n*ln(p) / (ln(2)^2)
        num_bits = static_cast<BitIndex>(
            -static_cast<double>(expected_elements) * std::log(false_positive_rate) / (std::log(2.0) * std::log(2.0)));

        // Optimal number of hash functions: k = (m/n) * ln(2)
        num_hash_functions = static_cast<NumHashes>(
            (static_cast<double>(num_bits) / expected_elements) * std::log(2.0));

        // Ensure at least 1 hash function
        if (num_hash_functions == 0)
            num_hash_functions = 1;

        bit_array.resize(num_bits, false);

        fmt::print("Bloom Filter created:\n");
        fmt::print("  Expected elements: {}\n", expected_elements);
        fmt::print("  Target false positive rate: {:.2f}%\n", false_positive_rate * 100);
        fmt::print("  Bit array size: {} bits ({:.2f} KB)\n",
                   num_bits, num_bits / 8192.0);
        fmt::print("  Number of hash functions: {}\n", num_hash_functions);
        fmt::print("  Bits per element: {:.2f}\n\n",
                   static_cast<double>(num_bits) / expected_elements);
    }

    // Insert element - O(k) where k = number of hash functions
    void insert(const std::string &item)
    {
        auto indices = getHashIndices(item);

        // Set all k bits to 1
        for (BitIndex index : indices)
        {
            bit_array[index] = true;
        }

        element_count++;
    }

    // Check if element might be in set - O(k)
    // Returns: true = "possibly in set" (might be false positive)
    //          false = "definitely not in set" (no false negatives)
    bool mightContain(const std::string &item) const
    {
        auto indices = getHashIndices(item);

        // Check if all k bits are set to 1
        for (BitIndex index : indices)
        {
            if (!bit_array[index])
            {
                return false; // Definitely not in set
            }
        }

        return true; // Possibly in set (or false positive)
    }

    // Calculate actual false positive probability based on current state
    // p = (1 - e^(-kn/m))^k
    // where k = hash functions, n = inserted elements, m = bit array size
    double getFalsePositiveRate() const
    {
        if (element_count == 0)
            return 0.0;

        double exponent = -static_cast<double>(num_hash_functions * element_count) / num_bits;
        double probability = std::pow(1.0 - std::exp(exponent),
                                      num_hash_functions);
        return probability;
    }

    // Get statistics
    void printStats() const
    {
        size_t bits_set = 0;
        for (bool bit : bit_array)
        {
            if (bit)
                bits_set++;
        }

        double fill_ratio = static_cast<double>(bits_set) / num_bits;

        fmt::print("\nBloom Filter Statistics:\n");
        fmt::print("  Elements inserted: {}\n", element_count);
        fmt::print("  Bits set: {} / {} ({:.2f}%)\n",
                   bits_set, num_bits, fill_ratio * 100);
        fmt::print("  Current false positive rate: {:.4f}%\n",
                   getFalsePositiveRate() * 100);
    }

    // Clear all bits - reset filter
    void clear()
    {
        std::fill(bit_array.begin(), bit_array.end(), false);
        element_count = 0;
    }
};

int main()
{
    fmt::print("=== BLOOM FILTER EXAMPLE ===\n\n");

    // Create bloom filter for ~100 elements with 1% false positive rate
    BloomFilter filter(100, 0.01);

    // 1. INSERT ELEMENTS
    fmt::print("1. INSERTING ELEMENTS\n");
    fmt::print("---------------------\n");

    std::vector<std::string> inserted_items = {
        "apple", "banana", "cherry", "date", "elderberry",
        "fig", "grape", "honeydew", "kiwi", "lemon",
        "mango", "nectarine", "orange", "papaya", "quince"};

    for (const auto &item : inserted_items)
    {
        filter.insert(item);
        fmt::print("Inserted: {}\n", item);
    }
    fmt::print("\n");

    // 2. CHECK MEMBERSHIP (should all return true)
    fmt::print("2. CHECKING INSERTED ELEMENTS (should all be found)\n");
    fmt::print("---------------------------------------------------\n");
    for (const auto &item : inserted_items)
    {
        bool found = filter.mightContain(item);
        fmt::print("Check '{}': {} (expected: YES)\n",
                   item, found ? "POSSIBLY IN SET" : "NOT IN SET");
    }
    fmt::print("\n");

    // 3. CHECK NON-EXISTENT ITEMS (should all return false, but might have false positives)
    fmt::print("3. CHECKING NON-EXISTENT ELEMENTS\n");
    fmt::print("----------------------------------\n");

    std::vector<std::string> non_existent = {
        "strawberry", "watermelon", "raspberry", "blueberry",
        "peach", "pear", "plum", "apricot", "coconut", "dragonfruit"};

    size_t false_positives = 0;
    for (const auto &item : non_existent)
    {
        bool found = filter.mightContain(item);
        fmt::print("Check '{}': {} (expected: NO)\n",
                   item, found ? "POSSIBLY IN SET" : "NOT IN SET");
        if (found)
        {
            false_positives++;
            fmt::print("  ^ FALSE POSITIVE detected!\n");
        }
    }
    fmt::print("\n");
    fmt::print("False positives: {} / {} ({:.1f}%)\n",
               false_positives, non_existent.size(),
               100.0 * false_positives / non_existent.size());

    // 4. STATISTICS
    filter.printStats();

    // 5. DEMONSTRATE SPACE EFFICIENCY
    fmt::print("\n\n4. SPACE EFFICIENCY COMPARISON\n");
    fmt::print("------------------------------\n");

    // Calculate space for different data structures
    size_t bloom_size = filter.getFalsePositiveRate() > 0 ? (100 * 10) / 8 : 0; // Approximation
    size_t set_size = inserted_items.size() * sizeof(std::string) +
                      inserted_items.size() * 20; // Approx string overhead

    fmt::print("Bloom Filter: ~{} bytes\n", bloom_size);
    fmt::print("std::unordered_set: ~{} bytes\n", set_size);
    fmt::print("Space savings: ~{:.1f}x\n",
               static_cast<double>(set_size) / bloom_size);

    // 6. USE CASE EXAMPLE: URL visited tracking
    fmt::print("\n\n5. USE CASE: WEB CRAWLER (URL VISITED TRACKING)\n");
    fmt::print("-----------------------------------------------\n");

    BloomFilter url_filter(1000000, 0.001); // 1M URLs, 0.1% FPR

    // Simulate crawling
    url_filter.insert("https://example.com/page1");
    url_filter.insert("https://example.com/page2");
    url_filter.insert("https://example.com/page3");

    fmt::print("Checking already visited URLs:\n");
    fmt::print("  https://example.com/page1: {}\n",
               url_filter.mightContain("https://example.com/page1") ? "Already visited" : "Not visited");
    fmt::print("  https://example.com/page4: {}\n",
               url_filter.mightContain("https://example.com/page4") ? "Already visited" : "Not visited");

    fmt::print("\nFor 1M URLs:\n");
    fmt::print("  Bloom filter: ~1.2 MB\n");
    fmt::print("  Hash set: ~40-80 MB\n");
    fmt::print("  Savings: 30-60x less memory!\n");

    return 0;
}