#include <algorithm>
#include <cstdint>
#include <fmt/core.h>
#include <fmt/format.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ============================================================================
// MULTI-BIT TRIE (MTrie) for IPv4 Routing Table
// ============================================================================
// Uses k-bit stride (typically 4, 8, or 16 bits per level)
// Reduces tree height: For 4-bit stride, IPv4 requires only 32/4 = 8 levels
// ============================================================================

// Type aliases for semantic clarity
using IPv4Address = uint32_t;
using PrefixLength = uint8_t; // 0-32 for IPv4
using BitPosition = uint8_t;  // 0-31 for IPv4
using TreeLevel = uint8_t;    // Level in the tree
using StrideSize = uint8_t;   // Number of bits per stride
using ChildIndex = uint16_t;  // Index into children array (0-255 for 8-bit stride)
using NumChildren = uint16_t; // Number of children (2^stride)

struct RouteInfo
{
    IPv4Address next_hop;
    std::string interface;
    PrefixLength prefix_length; // Store original prefix length

    RouteInfo(IPv4Address nh = 0, const std::string &iface = "", PrefixLength plen = 0)
        : next_hop(nh), interface(iface), prefix_length(plen) {}
};

class MTrieNode
{
public:
    static constexpr StrideSize STRIDE = 4;                  // 4 bits per level (16 children)
    static constexpr NumChildren NUM_CHILDREN = 1 << STRIDE; // 2^4 = 16

    std::unique_ptr<MTrieNode> children[NUM_CHILDREN];
    std::optional<RouteInfo> route_info;

    MTrieNode()
    {
        for (ChildIndex i = 0; i < NUM_CHILDREN; i++)
        {
            children[i] = nullptr;
        }
    }

    bool isLeaf() const
    {
        for (ChildIndex i = 0; i < NUM_CHILDREN; i++)
        {
            if (children[i])
                return false;
        }
        return true;
    }
};

class RoutingMTrie
{
private:
    std::unique_ptr<MTrieNode> root;
    static constexpr StrideSize STRIDE = MTrieNode::STRIDE;
    static constexpr NumChildren NUM_CHILDREN = MTrieNode::NUM_CHILDREN;
    static constexpr PrefixLength MAX_PREFIX_LENGTH = 32;

    // Extract 'num_bits' bits starting at 'position' from left
    static ChildIndex extractBits(IPv4Address ip, BitPosition position, StrideSize num_bits)
    {
        const BitPosition shift = MAX_PREFIX_LENGTH - position - num_bits;
        const uint32_t mask = (1u << num_bits) - 1;
        return static_cast<ChildIndex>((ip >> shift) & mask);
    }

    void printTrieHelper(const MTrieNode *node, IPv4Address prefix, TreeLevel level,
                         std::vector<std::string> &results) const
    {
        if (!node)
            return;

        if (node->route_info.has_value())
        {
            const PrefixLength actual_depth = node->route_info->prefix_length;
            std::string route = fmt::format("{}/{} -> {} ({})",
                                            ipToString(prefix),
                                            actual_depth,
                                            ipToString(node->route_info->next_hop),
                                            node->route_info->interface);
            results.push_back(route);
        }

        for (ChildIndex i = 0; i < NUM_CHILDREN; i++)
        {
            if (node->children[i])
            {
                const IPv4Address new_prefix = prefix |
                                               (static_cast<IPv4Address>(i) << (MAX_PREFIX_LENGTH - (level + 1) * STRIDE));
                printTrieHelper(node->children[i].get(), new_prefix, level + 1, results);
            }
        }
    }

public:
    RoutingMTrie() : root(std::make_unique<MTrieNode>()) {}

    static std::string ipToString(IPv4Address ip)
    {
        return fmt::format("{}.{}.{}.{}",
                           (ip >> 24) & 0xFF,
                           (ip >> 16) & 0xFF,
                           (ip >> 8) & 0xFF,
                           ip & 0xFF);
    }

    // Route Insertion: O(k/stride) - Much faster than basic trie!
    void insertRoute(IPv4Address prefix, PrefixLength prefix_length,
                     IPv4Address next_hop, const std::string &interface)
    {
        fmt::print("Inserting: {}/{} -> {} ({})\n",
                   ipToString(prefix), prefix_length,
                   ipToString(next_hop), interface);

        MTrieNode *current = root.get();
        BitPosition bits_processed = 0;

        while (bits_processed < prefix_length)
        {
            const StrideSize bits_to_extract =
                std::min(static_cast<StrideSize>(STRIDE),
                         static_cast<StrideSize>(prefix_length - bits_processed));
            const ChildIndex index = extractBits(prefix, bits_processed, bits_to_extract);

            // If this is a partial stride (less than STRIDE bits), we need to
            // expand to all possible child indices
            if (bits_to_extract < STRIDE)
            {
                // This prefix covers multiple children at this level
                const NumChildren num_covered = 1 << (STRIDE - bits_to_extract);
                const ChildIndex base_index = index << (STRIDE - bits_to_extract);

                // Install route info at current node for path compression
                // (prefix ends in middle of stride)
                current->route_info = RouteInfo(next_hop, interface, prefix_length);

                fmt::print("  Partial stride at level {}: covers {} children (base: {})\n",
                           bits_processed / STRIDE, num_covered, base_index);
                return;
            }

            if (!current->children[index])
            {
                current->children[index] = std::make_unique<MTrieNode>();
            }

            current = current->children[index].get();
            bits_processed += STRIDE;
        }

        current->route_info = RouteInfo(next_hop, interface, prefix_length);
    }

    // Route Search: O(k/stride)
    bool searchRoute(IPv4Address prefix, PrefixLength prefix_length) const
    {
        const MTrieNode *current = root.get();
        BitPosition bits_processed = 0;

        while (bits_processed < prefix_length)
        {
            const StrideSize bits_to_extract =
                std::min(static_cast<StrideSize>(STRIDE),
                         static_cast<StrideSize>(prefix_length - bits_processed));
            const ChildIndex index = extractBits(prefix, bits_processed, bits_to_extract);

            if (bits_to_extract < STRIDE)
            {
                // Partial stride - check if route exists at this node
                return current->route_info.has_value() &&
                       current->route_info->prefix_length == prefix_length;
            }

            if (!current->children[index])
            {
                return false;
            }

            current = current->children[index].get();
            bits_processed += STRIDE;
        }

        return current->route_info.has_value() &&
               current->route_info->prefix_length == prefix_length;
    }

    // Route Lookup using LPM: O(k/stride)
    // For 4-bit stride: only 8 levels for IPv4 vs 32 for basic trie!
    std::optional<RouteInfo> lookupLPM(IPv4Address ip_address) const
    {
        const MTrieNode *current = root.get();
        std::optional<RouteInfo> best_match;
        BitPosition bits_processed = 0;

        fmt::print("Looking up: {}\n", ipToString(ip_address));

        while (bits_processed < MAX_PREFIX_LENGTH)
        {
            if (current->route_info.has_value())
            {
                best_match = current->route_info;
                fmt::print("  Match at level {} (prefix /{}): {}\n",
                           bits_processed / STRIDE,
                           best_match->prefix_length,
                           ipToString(best_match->next_hop));
            }

            const ChildIndex index = extractBits(ip_address, bits_processed, STRIDE);

            if (!current->children[index])
            {
                break;
            }

            current = current->children[index].get();
            bits_processed += STRIDE;
        }

        // Final check
        if (current->route_info.has_value())
        {
            best_match = current->route_info;
            fmt::print("  Final match (prefix /{}): {}\n",
                       best_match->prefix_length,
                       ipToString(best_match->next_hop));
        }

        return best_match;
    }

    // Route Deletion: O(k/stride)
    bool deleteRoute(IPv4Address prefix, PrefixLength prefix_length)
    {
        fmt::print("Deleting: {}/{}\n", ipToString(prefix), prefix_length);
        return deleteRouteHelper(root.get(), prefix, prefix_length, 0);
    }

private:
    bool deleteRouteHelper(MTrieNode *node, IPv4Address prefix,
                           PrefixLength prefix_length, BitPosition bits_processed)
    {
        if (!node)
            return false;

        if (bits_processed >= prefix_length)
        {
            if (!node->route_info.has_value() ||
                node->route_info->prefix_length != prefix_length)
            {
                return false;
            }
            node->route_info.reset();
            return node->isLeaf();
        }

        const StrideSize bits_to_extract =
            std::min(static_cast<StrideSize>(STRIDE),
                     static_cast<StrideSize>(prefix_length - bits_processed));

        if (bits_to_extract < STRIDE)
        {
            // Partial stride deletion
            if (node->route_info.has_value() &&
                node->route_info->prefix_length == prefix_length)
            {
                node->route_info.reset();
                return node->isLeaf();
            }
            return false;
        }

        const ChildIndex index = extractBits(prefix, bits_processed, STRIDE);

        if (!node->children[index])
        {
            return false;
        }

        const bool shouldDeleteChild = deleteRouteHelper(
            node->children[index].get(), prefix, prefix_length,
            bits_processed + STRIDE);

        if (shouldDeleteChild)
        {
            node->children[index].reset();
        }

        return node->isLeaf() && !node->route_info.has_value();
    }

public:
    void printRoutes() const
    {
        std::vector<std::string> routes;
        printTrieHelper(root.get(), 0, 0, routes);
        fmt::print("\n=== Routing Table (MTrie) ===\n");
        for (const auto &route : routes)
        {
            fmt::print("{}\n", route);
        }
        fmt::print("==============================\n\n");
    }

    void printStatistics() const
    {
        const TreeLevel max_depth = (MAX_PREFIX_LENGTH + STRIDE - 1) / STRIDE;
        fmt::print("MTrie Statistics:\n");
        fmt::print("  Stride: {} bits\n", STRIDE);
        fmt::print("  Children per node: {}\n", NUM_CHILDREN);
        fmt::print("  Max depth for IPv4: {} levels\n", max_depth);
        fmt::print("  (vs 32 levels for 1-bit trie)\n");
        fmt::print("  Memory per node: {} bytes\n\n",
                   NUM_CHILDREN * sizeof(void *) + sizeof(RouteInfo));
    }
};

// ============================================================================
// EXAMPLE USAGE AND COMPARISON
// ============================================================================

int main()
{
    auto ipToInt = [](uint8_t a, uint8_t b, uint8_t c, uint8_t d) -> IPv4Address
    {
        return (static_cast<IPv4Address>(a) << 24) |
               (static_cast<IPv4Address>(b) << 16) |
               (static_cast<IPv4Address>(c) << 8) |
               static_cast<IPv4Address>(d);
    };

    fmt::print("=== MULTI-BIT TRIE (MTrie) EXAMPLE FOR IPv4 ROUTING ===\n\n");

    RoutingMTrie mtrie;
    mtrie.printStatistics();

    // 1. ROUTE INSERTION
    fmt::print("1. INSERTING ROUTES\n");
    fmt::print("-------------------\n");
    mtrie.insertRoute(ipToInt(192, 168, 0, 0), 16, ipToInt(10, 0, 0, 1), "eth0");
    mtrie.insertRoute(ipToInt(192, 168, 1, 0), 24, ipToInt(10, 0, 0, 2), "eth1");
    mtrie.insertRoute(ipToInt(192, 168, 1, 128), 25, ipToInt(10, 0, 0, 3), "eth2");
    mtrie.insertRoute(ipToInt(192, 168, 1, 192), 26, ipToInt(10, 0, 0, 4), "eth3");
    mtrie.insertRoute(ipToInt(10, 0, 0, 0), 8, ipToInt(172, 16, 0, 1), "eth4");
    mtrie.insertRoute(ipToInt(172, 16, 0, 0), 12, ipToInt(192, 168, 0, 1), "eth5");
    mtrie.insertRoute(ipToInt(0, 0, 0, 0), 0, ipToInt(8, 8, 8, 8), "default");
    fmt::print("\n");

    mtrie.printRoutes();

    // 2. ROUTE SEARCH
    fmt::print("2. SEARCHING FOR EXACT ROUTES\n");
    fmt::print("------------------------------\n");
    fmt::print("Search 192.168.1.0/24: {}\n",
               mtrie.searchRoute(ipToInt(192, 168, 1, 0), 24) ? "FOUND" : "NOT FOUND");
    fmt::print("Search 192.168.1.0/25: {}\n",
               mtrie.searchRoute(ipToInt(192, 168, 1, 0), 25) ? "FOUND" : "NOT FOUND");
    fmt::print("Search 192.168.2.0/24: {}\n",
               mtrie.searchRoute(ipToInt(192, 168, 2, 0), 24) ? "FOUND" : "NOT FOUND");
    fmt::print("\n");

    // 3. LPM LOOKUPS
    fmt::print("3. LONGEST PREFIX MATCH (LPM) LOOKUPS\n");
    fmt::print("--------------------------------------\n");

    auto testLookup = [&](IPv4Address ip)
    {
        auto result = mtrie.lookupLPM(ip);
        if (result)
        {
            fmt::print("  => Route to: {} via {}\n\n",
                       mtrie.ipToString(result->next_hop), result->interface);
        }
        else
        {
            fmt::print("  => No route found\n\n");
        }
    };

    testLookup(ipToInt(192, 168, 1, 100)); // Should match /24
    testLookup(ipToInt(192, 168, 1, 200)); // Should match /26
    testLookup(ipToInt(192, 168, 5, 1));   // Should match /16
    testLookup(ipToInt(10, 50, 50, 50));   // Should match /8
    testLookup(ipToInt(172, 16, 5, 1));    // Should match /12
    testLookup(ipToInt(8, 8, 4, 4));       // Should match default

    // 4. ROUTE DELETION
    fmt::print("4. DELETING ROUTES\n");
    fmt::print("------------------\n");
    mtrie.deleteRoute(ipToInt(192, 168, 1, 192), 26);
    fmt::print("\n");

    mtrie.printRoutes();

    fmt::print("After deletion, lookup 192.168.1.200 again:\n");
    testLookup(ipToInt(192, 168, 1, 200)); // Should now match /25

    // Performance comparison
    fmt::print("\n=== PERFORMANCE COMPARISON ===\n");
    fmt::print("Operation complexity (k = 32 for IPv4):\n\n");
    fmt::print("Basic Trie (1-bit stride):\n");
    fmt::print("  - Levels: 32\n");
    fmt::print("  - Memory per node: 2 pointers + route info\n");
    fmt::print("  - Cache efficiency: Poor (deep tree)\n\n");

    fmt::print("MTrie (4-bit stride):\n");
    fmt::print("  - Levels: 8 (4x reduction!)\n");
    fmt::print("  - Memory per node: 16 pointers + route info\n");
    fmt::print("  - Cache efficiency: Better (shallow tree)\n");
    fmt::print("  - Lookup speed: ~4x faster\n\n");

    fmt::print("MTrie (8-bit stride):\n");
    fmt::print("  - Levels: 4 (8x reduction!)\n");
    fmt::print("  - Memory per node: 256 pointers + route info\n");
    fmt::print("  - Cache efficiency: Excellent (very shallow)\n");
    fmt::print("  - Lookup speed: ~8x faster\n");
    fmt::print("  - Trade-off: Higher memory usage per node\n\n");

    return 0;
}