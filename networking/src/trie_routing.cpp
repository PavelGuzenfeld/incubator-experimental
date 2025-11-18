#include <cstdint>
#include <fmt/core.h>
#include <fmt/format.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ============================================================================
// BASIC TRIE (1-bit stride) for IPv4 Routing Table
// ============================================================================

// Type aliases for semantic clarity
using IPv4Address = uint32_t;
using PrefixLength = uint8_t; // 0-32 for IPv4
using BitPosition = uint8_t;  // 0-31 for IPv4
using TreeDepth = uint8_t;    // 0-32 for tree traversal
using ChildIndex = uint8_t;   // 0 or 1 for binary trie
using BitValue = uint8_t;     // 0 or 1

struct RouteInfo
{
    IPv4Address next_hop;
    std::string interface;

    RouteInfo(IPv4Address nh = 0, const std::string &iface = "")
        : next_hop(nh), interface(iface) {}
};

class TrieNode
{
public:
    static constexpr size_t NUM_CHILDREN = 2;
    std::unique_ptr<TrieNode> children[NUM_CHILDREN]; // 0 and 1
    std::optional<RouteInfo> route_info;

    TrieNode() : children{nullptr, nullptr} {}

    bool isLeaf() const
    {
        return !children[0] && !children[1];
    }
};

class RoutingTrie
{
private:
    std::unique_ptr<TrieNode> root;

    static constexpr PrefixLength MAX_PREFIX_LENGTH = 32;

    // Helper: Get bit at position from left (0-indexed from left)
    static BitValue getBit(IPv4Address ip, BitPosition position)
    {
        return static_cast<BitValue>((ip >> (31 - position)) & 1);
    }

    // Helper for printing
    void printTrieHelper(const TrieNode *node, IPv4Address prefix, TreeDepth depth,
                         std::vector<std::string> &results) const
    {
        if (!node)
            return;

        if (node->route_info.has_value())
        {
            std::string route = fmt::format("{}/{} -> {} ({})",
                                            ipToString(prefix),
                                            depth,
                                            ipToString(node->route_info->next_hop),
                                            node->route_info->interface);
            results.push_back(route);
        }

        if (node->children[0])
        {
            printTrieHelper(node->children[0].get(), prefix, depth + 1, results);
        }
        if (node->children[1])
        {
            IPv4Address new_prefix = prefix | (1u << (31 - depth));
            printTrieHelper(node->children[1].get(), new_prefix, depth + 1, results);
        }
    }

public:
    RoutingTrie() : root(std::make_unique<TrieNode>()) {}

    // Helper: Convert IP to string
    static std::string ipToString(IPv4Address ip)
    {
        return fmt::format("{}.{}.{}.{}",
                           (ip >> 24) & 0xFF,
                           (ip >> 16) & 0xFF,
                           (ip >> 8) & 0xFF,
                           ip & 0xFF);
    }

    // Route Insertion: O(k) where k = prefix_length
    void insertRoute(IPv4Address prefix, PrefixLength prefix_length,
                     IPv4Address next_hop, const std::string &interface)
    {
        fmt::print("Inserting: {}/{} -> {} ({})\n",
                   ipToString(prefix), prefix_length,
                   ipToString(next_hop), interface);

        TrieNode *current = root.get();

        // Traverse/create path for prefix_length bits
        for (BitPosition i = 0; i < prefix_length; i++)
        {
            ChildIndex bit = getBit(prefix, i);

            if (!current->children[bit])
            {
                current->children[bit] = std::make_unique<TrieNode>();
            }
            current = current->children[bit].get();
        }

        // Store route info at this prefix
        current->route_info = RouteInfo(next_hop, interface);
    }

    // Route Search: O(k) - Check if exact prefix exists
    bool searchRoute(IPv4Address prefix, PrefixLength prefix_length) const
    {
        const TrieNode *current = root.get();

        for (BitPosition i = 0; i < prefix_length; i++)
        {
            ChildIndex bit = getBit(prefix, i);
            if (!current->children[bit])
            {
                return false;
            }
            current = current->children[bit].get();
        }

        return current->route_info.has_value();
    }

    // Route Lookup using LPM (Longest Prefix Match): O(k)
    std::optional<RouteInfo> lookupLPM(IPv4Address ip_address) const
    {
        const TrieNode *current = root.get();
        std::optional<RouteInfo> best_match;

        fmt::print("Looking up: {}\n", ipToString(ip_address));

        // Traverse as far as possible, keeping track of last valid route
        for (BitPosition i = 0; i < MAX_PREFIX_LENGTH; i++)
        {
            // Update best match if current node has a route
            if (current->route_info.has_value())
            {
                best_match = current->route_info;
                fmt::print("  Found match at depth {}: {}\n",
                           i, ipToString(best_match->next_hop));
            }

            ChildIndex bit = getBit(ip_address, i);
            if (!current->children[bit])
            {
                break; // Can't go further
            }
            current = current->children[bit].get();
        }

        // Check one last time after loop
        if (current->route_info.has_value())
        {
            best_match = current->route_info;
        }

        return best_match;
    }

    // Route Deletion: O(k)
    bool deleteRoute(IPv4Address prefix, PrefixLength prefix_length)
    {
        fmt::print("Deleting: {}/{}\n", ipToString(prefix), prefix_length);
        return deleteRouteHelper(root.get(), prefix, prefix_length, 0);
    }

private:
    bool deleteRouteHelper(TrieNode *node, IPv4Address prefix,
                           PrefixLength prefix_length, TreeDepth depth)
    {
        if (!node)
            return false;

        // Reached the prefix node
        if (depth == prefix_length)
        {
            if (!node->route_info.has_value())
            {
                return false; // Route doesn't exist
            }
            node->route_info.reset(); // Remove route info
            return node->isLeaf();    // Return true if can be deleted
        }

        // Recurse down
        ChildIndex bit = getBit(prefix, depth);
        if (!node->children[bit])
        {
            return false; // Path doesn't exist
        }

        bool shouldDeleteChild = deleteRouteHelper(
            node->children[bit].get(), prefix, prefix_length, depth + 1);

        // Delete child if it's now empty
        if (shouldDeleteChild)
        {
            node->children[bit].reset();
        }

        // This node can be deleted if it's a leaf with no route info
        return node->isLeaf() && !node->route_info.has_value();
    }

public:
    void printRoutes() const
    {
        std::vector<std::string> routes;
        printTrieHelper(root.get(), 0, 0, routes);
        fmt::print("\n=== Routing Table ===\n");
        for (const auto &route : routes)
        {
            fmt::print("{}\n", route);
        }
        fmt::print("=====================\n\n");
    }
};

// ============================================================================
// EXAMPLE USAGE
// ============================================================================

int main()
{
    RoutingTrie trie;

    fmt::print("=== BASIC TRIE EXAMPLE FOR IPv4 ROUTING ===\n\n");

    // Helper to convert dotted IP to IPv4Address
    auto ipToInt = [](uint8_t a, uint8_t b, uint8_t c, uint8_t d) -> IPv4Address
    {
        return (static_cast<IPv4Address>(a) << 24) |
               (static_cast<IPv4Address>(b) << 16) |
               (static_cast<IPv4Address>(c) << 8) |
               static_cast<IPv4Address>(d);
    };

    // 1. ROUTE INSERTION - O(k)
    fmt::print("1. INSERTING ROUTES\n");
    fmt::print("-------------------\n");
    trie.insertRoute(ipToInt(192, 168, 0, 0), 16, ipToInt(10, 0, 0, 1), "eth0");
    trie.insertRoute(ipToInt(192, 168, 1, 0), 24, ipToInt(10, 0, 0, 2), "eth1");
    trie.insertRoute(ipToInt(192, 168, 1, 128), 25, ipToInt(10, 0, 0, 3), "eth2");
    trie.insertRoute(ipToInt(10, 0, 0, 0), 8, ipToInt(172, 16, 0, 1), "eth3");
    trie.insertRoute(ipToInt(0, 0, 0, 0), 0, ipToInt(8, 8, 8, 8), "default");
    fmt::print("\n");

    trie.printRoutes();

    // 2. ROUTE SEARCH - O(k)
    fmt::print("2. SEARCHING FOR EXACT ROUTES\n");
    fmt::print("------------------------------\n");
    fmt::print("Search 192.168.1.0/24: {}\n",
               trie.searchRoute(ipToInt(192, 168, 1, 0), 24) ? "FOUND" : "NOT FOUND");
    fmt::print("Search 192.168.2.0/24: {}\n",
               trie.searchRoute(ipToInt(192, 168, 2, 0), 24) ? "FOUND" : "NOT FOUND");
    fmt::print("\n");

    // 3. ROUTE LOOKUP (LPM) - O(k)
    fmt::print("3. LONGEST PREFIX MATCH (LPM) LOOKUPS\n");
    fmt::print("--------------------------------------\n");

    auto testLookup = [&](IPv4Address ip)
    {
        auto result = trie.lookupLPM(ip);
        if (result)
        {
            fmt::print("  -> Route to: {} via {}\n\n",
                       trie.ipToString(result->next_hop), result->interface);
        }
        else
        {
            fmt::print("  -> No route found\n\n");
        }
    };

    testLookup(ipToInt(192, 168, 1, 100)); // Should match /24
    testLookup(ipToInt(192, 168, 1, 200)); // Should match /25
    testLookup(ipToInt(192, 168, 5, 1));   // Should match /16
    testLookup(ipToInt(10, 0, 5, 1));      // Should match /8
    testLookup(ipToInt(8, 8, 4, 4));       // Should match default

    // 4. ROUTE DELETION - O(k)
    fmt::print("4. DELETING ROUTES\n");
    fmt::print("------------------\n");
    trie.deleteRoute(ipToInt(192, 168, 1, 128), 25);
    fmt::print("\n");

    trie.printRoutes();

    fmt::print("After deletion, lookup 192.168.1.200 again:\n");
    testLookup(ipToInt(192, 168, 1, 200)); // Now should match /24, not /25

    return 0;
}