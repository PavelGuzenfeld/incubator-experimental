// SPF Algorithm with ECMP (Equal-Cost Multi-Path) Support
// Compiler Explorer: Use C++17, add flag: -lfmt
// Handles multiple equal-cost paths to same destination

#include <cstdint>
#include <fmt/core.h>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using RouterID = uint32_t;
using Cost = uint32_t;
using SequenceNum = uint32_t;

constexpr Cost INFINITE_COST = std::numeric_limits<Cost>::max();

// Link State Advertisement (LSA)
struct LSA
{
    RouterID originating_router;
    SequenceNum sequence_number;
    std::unordered_map<RouterID, Cost> neighbors;

    LSA() : originating_router(0), sequence_number(0) {}
    LSA(RouterID router_id, SequenceNum seq = 0)
        : originating_router(router_id), sequence_number(seq) {}

    void addLink(RouterID neighbor, Cost cost)
    {
        neighbors[neighbor] = cost;
    }
};

// Link State Database
class LSDB
{
private:
    std::unordered_map<RouterID, LSA> database;

public:
    bool installLSA(const LSA &lsa)
    {
        auto it = database.find(lsa.originating_router);
        if (it == database.end() ||
            lsa.sequence_number > it->second.sequence_number)
        {
            database[lsa.originating_router] = lsa;
            return true;
        }
        return false;
    }

    const LSA *getLSA(RouterID router_id) const
    {
        auto it = database.find(router_id);
        return (it != database.end()) ? &it->second : nullptr;
    }

    std::vector<RouterID> getAllRouters() const
    {
        std::vector<RouterID> routers;
        for (const auto &[router_id, lsa] : database)
        {
            routers.push_back(router_id);
        }
        return routers;
    }
};

// SPF Node with ECMP support - stores MULTIPLE next hops
struct SPFNode
{
    Cost distance;
    std::vector<RouterID> next_hops;    // Multiple next hops for ECMP!
    std::vector<RouterID> predecessors; // Multiple predecessors

    SPFNode() : distance(INFINITE_COST) {}
};

// Priority queue element
struct PQElement
{
    Cost cost;
    RouterID router_id;

    PQElement(Cost c, RouterID id) : cost(c), router_id(id) {}

    bool operator>(const PQElement &other) const
    {
        return cost > other.cost;
    }
};

// SPF Algorithm with ECMP support
std::unordered_map<RouterID, SPFNode> runSPF_ECMP(const LSDB &lsdb, RouterID source)
{
    std::unordered_map<RouterID, SPFNode> spf_tree;

    fmt::print("Running SPF with ECMP from Router {}...\n\n", source);

    // Initialize source
    spf_tree[source].distance = 0;
    spf_tree[source].next_hops.push_back(source);

    std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>>
        pq;
    pq.emplace(0, source);

    std::unordered_map<RouterID, bool> visited_map;
    size_t step = 1;

    while (!pq.empty())
    {
        PQElement current = pq.top();
        pq.pop();

        RouterID current_router = current.router_id;
        Cost current_cost = current.cost;

        // Skip if we've already found a better path
        if (spf_tree.find(current_router) != spf_tree.end() &&
            current_cost > spf_tree[current_router].distance)
        {
            continue;
        }

        if (visited_map[current_router])
            continue;
        visited_map[current_router] = true;

        fmt::print("Step {}: Examining Router {} (cost: {})\n",
                   step++, current_router, current_cost);

        const LSA *lsa = lsdb.getLSA(current_router);
        if (!lsa)
            continue;

        // Examine all neighbors
        for (const auto &[neighbor_id, link_cost] : lsa->neighbors)
        {
            Cost new_cost = current_cost + link_cost;

            if (spf_tree.find(neighbor_id) == spf_tree.end())
            {
                spf_tree[neighbor_id] = SPFNode();
            }

            // ECMP Logic: Handle equal-cost paths!
            if (new_cost < spf_tree[neighbor_id].distance)
            {
                // Found better path - replace all previous paths
                fmt::print("  Better path to Router {}: old={}, new={}\n",
                           neighbor_id,
                           spf_tree[neighbor_id].distance == INFINITE_COST ? std::string("INF") : std::to_string(spf_tree[neighbor_id].distance),
                           new_cost);

                spf_tree[neighbor_id].distance = new_cost;
                spf_tree[neighbor_id].predecessors.clear();
                spf_tree[neighbor_id].predecessors.push_back(current_router);
                spf_tree[neighbor_id].next_hops.clear();

                // Calculate next hop
                if (current_router == source)
                {
                    spf_tree[neighbor_id].next_hops.push_back(neighbor_id);
                }
                else
                {
                    spf_tree[neighbor_id].next_hops = spf_tree[current_router].next_hops;
                }

                pq.emplace(new_cost, neighbor_id);
            }
            else if (new_cost == spf_tree[neighbor_id].distance)
            {
                // ECMP: Equal-cost path found!
                fmt::print("  ECMP: Equal-cost path to Router {} (cost={})\n",
                           neighbor_id, new_cost);

                // Add this predecessor
                spf_tree[neighbor_id].predecessors.push_back(current_router);

                // Add next hop(s) if not already present
                if (current_router == source)
                {
                    // Direct connection - add neighbor as next hop
                    bool found = false;
                    for (auto nh : spf_tree[neighbor_id].next_hops)
                    {
                        if (nh == neighbor_id)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        spf_tree[neighbor_id].next_hops.push_back(neighbor_id);
                    }
                }
                else
                {
                    // Inherit next hops from current router
                    for (auto nh : spf_tree[current_router].next_hops)
                    {
                        bool found = false;
                        for (auto existing_nh : spf_tree[neighbor_id].next_hops)
                        {
                            if (existing_nh == nh)
                            {
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            spf_tree[neighbor_id].next_hops.push_back(nh);
                        }
                    }
                }
            }
        }
        fmt::print("\n");
    }

    return spf_tree;
}

// Routing table entry with ECMP support
struct RouteEntry
{
    RouterID destination;
    Cost cost;
    std::vector<RouterID> next_hops; // Multiple next hops for load balancing

    RouteEntry(RouterID dest, Cost c, const std::vector<RouterID> &nhs)
        : destination(dest), cost(c), next_hops(nhs) {}
};

// Build routing table with ECMP
std::vector<RouteEntry> buildRoutingTable(
    const std::unordered_map<RouterID, SPFNode> &spf_tree, RouterID source)
{

    std::vector<RouteEntry> routing_table;

    for (const auto &[dest, node] : spf_tree)
    {
        if (dest != source && node.distance != INFINITE_COST)
        {
            routing_table.emplace_back(dest, node.distance, node.next_hops);
        }
    }

    return routing_table;
}

// Print routing table with ECMP indication
void printRoutingTable(RouterID router_id, const std::vector<RouteEntry> &table)
{
    fmt::print("\n=== ROUTING TABLE FOR ROUTER {} (with ECMP) ===\n", router_id);
    fmt::print("Destination | Cost | Next Hops\n");
    fmt::print("------------|------|-----------\n");

    for (const auto &entry : table)
    {
        fmt::print("Router {:4} | {:4} | ", entry.destination, entry.cost);

        for (size_t i = 0; i < entry.next_hops.size(); i++)
        {
            fmt::print("Router {}", entry.next_hops[i]);
            if (i < entry.next_hops.size() - 1)
            {
                fmt::print(", ");
            }
        }

        // Mark ECMP routes
        if (entry.next_hops.size() > 1)
        {
            fmt::print(" [ECMP x{}]", entry.next_hops.size());
        }
        fmt::print("\n");
    }
    fmt::print("{}\n", std::string(50, '='));
}

// Router class
class Router
{
private:
    RouterID router_id;
    LSDB link_state_db;
    SequenceNum lsa_sequence;
    std::unordered_map<RouterID, Cost> local_neighbors;

public:
    Router(RouterID id) : router_id(id), lsa_sequence(0) {}

    RouterID getID() const { return router_id; }

    void addNeighbor(RouterID neighbor, Cost cost)
    {
        local_neighbors[neighbor] = cost;
    }

    LSA generateLSA()
    {
        lsa_sequence++;
        LSA lsa(router_id, lsa_sequence);
        for (const auto &[neighbor, cost] : local_neighbors)
        {
            lsa.addLink(neighbor, cost);
        }
        return lsa;
    }

    void receiveLSA(const LSA &lsa)
    {
        link_state_db.installLSA(lsa);
    }

    void calculateRoutingTable()
    {
        fmt::print("\nRouter {}: Calculating routing table with ECMP\n", router_id);
        fmt::print("{}\n", std::string(50, '='));

        auto spf_tree = runSPF_ECMP(link_state_db, router_id);
        auto routing_table = buildRoutingTable(spf_tree, router_id);
        printRoutingTable(router_id, routing_table);
    }
};

// Flood LSAs
void floodLSAs(std::vector<Router> &routers)
{
    std::vector<LSA> all_lsas;
    for (auto &router : routers)
    {
        all_lsas.push_back(router.generateLSA());
    }

    for (const auto &lsa : all_lsas)
    {
        for (auto &router : routers)
        {
            router.receiveLSA(lsa);
        }
    }
}

int main()
{
    fmt::print("=== SPF WITH ECMP (EQUAL-COST MULTI-PATH) ===\n\n");

    // Example topology with ECMP opportunities:
    //
    //       10          10
    //  R1 ------ R2 ------ R3
    //   |    \\  /    \\  /  |
    // 10|     \/      \/   |10
    //   |     /\      /\   |
    //   |    /  \\  /  \\  |
    //  R4 ------ R5 ------ R6
    //       10          10
    //
    // R1-R5: cost 10 (diagonal)
    // R2-R4: cost 10 (diagonal)
    // R3-R5: cost 10 (diagonal)

    fmt::print("Network Topology with ECMP opportunities:\n");
    fmt::print("       10          10\n");
    fmt::print("  R1 ------ R2 ------ R3\n");
    fmt::print("   |    \\\\  /    \\\\  /  |\n");
    fmt::print(" 10|   10\\\\/   10\\\\/   |10\n");
    fmt::print("   |      /\\      /\\   |\n");
    fmt::print("   |    /10\\  /10\\\\  |\n");
    fmt::print("  R4 ------ R5 ------ R6\n");
    fmt::print("       10          10\n\n");

    // Create routers
    std::vector<Router> routers;
    for (RouterID i = 1; i <= 6; i++)
    {
        routers.emplace_back(i);
    }

    // Configure symmetric topology with ECMP opportunities
    // Horizontal edges
    routers[0].addNeighbor(2, 10);
    routers[1].addNeighbor(1, 10); // R1-R2
    routers[1].addNeighbor(3, 10);
    routers[2].addNeighbor(2, 10); // R2-R3
    routers[3].addNeighbor(5, 10);
    routers[4].addNeighbor(4, 10); // R4-R5
    routers[4].addNeighbor(6, 10);
    routers[5].addNeighbor(5, 10); // R5-R6

    // Vertical edges
    routers[0].addNeighbor(4, 10);
    routers[3].addNeighbor(1, 10); // R1-R4
    routers[2].addNeighbor(6, 10);
    routers[5].addNeighbor(3, 10); // R3-R6

    // Diagonal edges (create ECMP opportunities)
    routers[0].addNeighbor(5, 10);
    routers[4].addNeighbor(1, 10); // R1-R5
    routers[1].addNeighbor(4, 10);
    routers[3].addNeighbor(2, 10); // R2-R4
    routers[1].addNeighbor(5, 10);
    routers[4].addNeighbor(2, 10); // R2-R5
    routers[2].addNeighbor(5, 10);
    routers[4].addNeighbor(3, 10); // R3-R5

    fmt::print("=== FLOODING LSAs ===\n");
    floodLSAs(routers);
    fmt::print("LSA flooding complete.\n\n");

    fmt::print("=== SPF CALCULATION WITH ECMP ===\n");

    // Calculate routing tables
    for (auto &router : routers)
    {
        router.calculateRoutingTable();
    }

    fmt::print("\n=== ECMP ANALYSIS ===\n");
    fmt::print("Router 1 to Router 6:\n");
    fmt::print("  Path 1: R1 -> R2 -> R3 -> R6 (cost: 30)\n");
    fmt::print("  Path 2: R1 -> R4 -> R5 -> R6 (cost: 30)\n");
    fmt::print("  Path 3: R1 -> R5 -> R6 (cost: 20) <- SHORTEST!\n");
    fmt::print("  SPF chooses cost=20 path via R5\n\n");

    fmt::print("Router 1 to Router 2:\n");
    fmt::print("  Path 1: R1 -> R2 (direct, cost: 10)\n");
    fmt::print("  Only one optimal path\n\n");

    fmt::print("Router 4 to Router 6:\n");
    fmt::print("  Path 1: R4 -> R5 -> R6 (cost: 20)\n");
    fmt::print("  Path 2: R4 -> R2 -> R3 -> R6 (cost: 30)\n");
    fmt::print("  SPF chooses cost=20 path via R5\n\n");

    fmt::print("=== KEY ECMP CONCEPTS ===\n");
    fmt::print("1. ECMP Detection:\n");
    fmt::print("   - When new_cost == existing_cost\n");
    fmt::print("   - Store multiple predecessors\n");
    fmt::print("   - Store multiple next hops\n\n");

    fmt::print("2. Load Balancing:\n");
    fmt::print("   - Traffic can be split across equal-cost paths\n");
    fmt::print("   - Per-flow or per-packet distribution\n");
    fmt::print("   - Improves bandwidth utilization\n\n");

    fmt::print("3. Routing Table:\n");
    fmt::print("   - Multiple next hops for same destination\n");
    fmt::print("   - Marked with [ECMP xN] where N = number of paths\n");
    fmt::print("   - Router can choose among them for load balancing\n");

    return 0;
}