// Dijkstra's Shortest Path Algorithm
// Compiler Explorer: Use C++17, add flag: -lfmt
// Finds shortest paths from source to all vertices in weighted graph
// Works with non-negative edge weights only

#include <algorithm>
#include <cstdint>
#include <fmt/core.h>
#include <limits>
#include <queue>
#include <string>
#include <vector>

using VertexID = uint32_t;
using Distance = uint32_t;
using EdgeWeight = uint32_t;

// Special value for infinity (unreachable vertex)
constexpr Distance INFINITE_DISTANCE = std::numeric_limits<Distance>::max();

// Edge representation: (destination vertex, weight)
struct Edge
{
    VertexID to;
    EdgeWeight weight;

    Edge(VertexID dest, EdgeWeight w) : to(dest), weight(w) {}
};

// Graph representation using adjacency list
class Graph
{
private:
    std::vector<std::vector<Edge>> adjacency_list;
    size_t num_vertices;

public:
    explicit Graph(size_t n) : num_vertices(n)
    {
        adjacency_list.resize(n);
    }

    // Add directed edge from u to v with given weight
    void addEdge(VertexID from, VertexID to, EdgeWeight weight)
    {
        adjacency_list[from].emplace_back(to, weight);
    }

    // Add undirected edge (bidirectional)
    void addUndirectedEdge(VertexID u, VertexID v, EdgeWeight weight)
    {
        addEdge(u, v, weight);
        addEdge(v, u, weight);
    }

    size_t getNumVertices() const { return num_vertices; }

    const std::vector<Edge> &getNeighbors(VertexID v) const
    {
        return adjacency_list[v];
    }
};

// Node in priority queue: (distance, vertex)
// Note: priority_queue is max-heap by default, so we use greater<> for min-heap
struct PQNode
{
    Distance distance;
    VertexID vertex;

    PQNode(Distance d, VertexID v) : distance(d), vertex(v) {}

    // Comparison for min-heap (smaller distance = higher priority)
    bool operator>(const PQNode &other) const
    {
        return distance > other.distance;
    }
};

// Dijkstra's algorithm result
struct DijkstraResult
{
    std::vector<Distance> distances;    // distances[v] = shortest distance from source to v
    std::vector<VertexID> predecessors; // predecessors[v] = previous vertex in shortest path

    DijkstraResult(size_t n)
        : distances(n, INFINITE_DISTANCE),
          predecessors(n, INFINITE_DISTANCE) {}
};

// Dijkstra's Algorithm - O((V + E) log V) with binary heap
// V = number of vertices, E = number of edges
DijkstraResult dijkstra(const Graph &graph, VertexID source)
{
    const size_t n = graph.getNumVertices();
    DijkstraResult result(n);

    // Min-heap priority queue: (distance, vertex)
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;

    // Initialize source vertex
    result.distances[source] = 0;
    pq.emplace(0, source);

    fmt::print("Starting Dijkstra from vertex {}\n\n", source);

    // Track which vertices have been processed
    std::vector<bool> visited(n, false);

    size_t step = 1;

    while (!pq.empty())
    {
        // Extract vertex with minimum distance
        PQNode current = pq.top();
        pq.pop();

        VertexID u = current.vertex;
        Distance dist_u = current.distance;

        // Skip if already processed (outdated entry in PQ)
        if (visited[u])
            continue;

        visited[u] = true;

        fmt::print("Step {}: Processing vertex {} (distance: {})\n",
                   step++, u, dist_u);

        // Relaxation: Check all neighbors
        for (const Edge &edge : graph.getNeighbors(u))
        {
            VertexID v = edge.to;
            EdgeWeight weight = edge.weight;
            Distance new_distance = dist_u + weight;

            // If found shorter path to v
            if (new_distance < result.distances[v])
            {
                fmt::print("  Relaxing edge {}->{}: old distance = {}, new distance = {}\n",
                           u, v,
                           result.distances[v] == INFINITE_DISTANCE ? std::string("INF") : std::to_string(result.distances[v]),
                           new_distance);

                result.distances[v] = new_distance;
                result.predecessors[v] = u;
                pq.emplace(new_distance, v);
            }
        }
        fmt::print("\n");
    }

    return result;
}

// Reconstruct path from source to destination
std::vector<VertexID> reconstructPath(const DijkstraResult &result,
                                      VertexID source, VertexID destination)
{
    std::vector<VertexID> path;

    // Check if destination is reachable
    if (result.distances[destination] == INFINITE_DISTANCE)
    {
        return path; // Empty path = unreachable
    }

    // Backtrack from destination to source
    VertexID current = destination;
    while (current != INFINITE_DISTANCE)
    {
        path.push_back(current);
        if (current == source)
            break;
        current = result.predecessors[current];
    }

    // Reverse to get path from source to destination
    std::reverse(path.begin(), path.end());
    return path;
}

// Print results
void printResults(const DijkstraResult &result, VertexID source)
{
    fmt::print("=== SHORTEST DISTANCES FROM VERTEX {} ===\n", source);

    for (size_t v = 0; v < result.distances.size(); v++)
    {
        fmt::print("Vertex {}: distance = ", v);

        if (result.distances[v] == INFINITE_DISTANCE)
        {
            fmt::print("UNREACHABLE\n");
        }
        else
        {
            fmt::print("{}", result.distances[v]);

            // Show path
            auto path = reconstructPath(result, source, v);
            fmt::print(", path: ");
            for (size_t i = 0; i < path.size(); i++)
            {
                fmt::print("{}", path[i]);
                if (i < path.size() - 1)
                    fmt::print(" -> ");
            }
            fmt::print("\n");
        }
    }
    fmt::print("\n");
}

int main()
{
    fmt::print("=== DIJKSTRA'S SHORTEST PATH ALGORITHM ===\n\n");

    // Example 1: Simple graph
    fmt::print("EXAMPLE 1: Simple weighted graph\n");
    fmt::print("--------------------------------\n");
    fmt::print("Graph structure:\n");
    fmt::print("       7        1\n");
    fmt::print("   0 ---- 1 -------- 2\n");
    fmt::print("   |      |\\         |\n");
    fmt::print("  5|     2| \\3       |2\n");
    fmt::print("   |      |  \\       |\n");
    fmt::print("   3 ---- 4   ------ 5\n");
    fmt::print("      1        4\n\n");

    Graph graph1(6); // 6 vertices (0-5)

    // Add edges (undirected graph)
    graph1.addUndirectedEdge(0, 1, 7);
    graph1.addUndirectedEdge(0, 3, 5);
    graph1.addUndirectedEdge(1, 2, 1);
    graph1.addUndirectedEdge(1, 4, 2);
    graph1.addUndirectedEdge(1, 5, 3);
    graph1.addUndirectedEdge(2, 5, 2);
    graph1.addUndirectedEdge(3, 4, 1);
    graph1.addUndirectedEdge(4, 5, 4);

    // Run Dijkstra from vertex 0
    auto result1 = dijkstra(graph1, 0);
    printResults(result1, 0);

    // Example 2: Directed graph with one-way streets
    fmt::print("\nEXAMPLE 2: Directed graph (road network)\n");
    fmt::print("-----------------------------------------\n");
    fmt::print("Graph structure (arrows show direction):\n");
    fmt::print("       4\n");
    fmt::print("   0 ----> 1\n");
    fmt::print("   |       |\n");
    fmt::print("  1|       |2\n");
    fmt::print("   v       v\n");
    fmt::print("   2 ----> 3\n");
    fmt::print("       1\n\n");

    Graph graph2(4); // 4 vertices

    // Add directed edges
    graph2.addEdge(0, 1, 4);
    graph2.addEdge(0, 2, 1);
    graph2.addEdge(1, 3, 2);
    graph2.addEdge(2, 3, 1);

    // Run Dijkstra from vertex 0
    auto result2 = dijkstra(graph2, 0);
    printResults(result2, 0);

    // Example 3: Graph with unreachable vertices
    fmt::print("\nEXAMPLE 3: Disconnected graph\n");
    fmt::print("-----------------------------\n");
    fmt::print("   0 -- 1    2 -- 3\n");
    fmt::print("  (connected) (disconnected)\n\n");

    Graph graph3(4);
    graph3.addUndirectedEdge(0, 1, 5);
    graph3.addUndirectedEdge(2, 3, 3);

    auto result3 = dijkstra(graph3, 0);
    printResults(result3, 0);

    // Example 4: Real-world scenario - City routing
    fmt::print("\nEXAMPLE 4: City routing (time in minutes)\n");
    fmt::print("------------------------------------------\n");

    // Cities: 0=Home, 1=School, 2=Mall, 3=Park, 4=Hospital, 5=Airport
    std::vector<std::string> locations = {
        "Home", "School", "Mall", "Park", "Hospital", "Airport"};

    Graph city(6);
    city.addUndirectedEdge(0, 1, 10); // Home - School: 10 min
    city.addUndirectedEdge(0, 2, 15); // Home - Mall: 15 min
    city.addUndirectedEdge(1, 2, 12); // School - Mall: 12 min
    city.addUndirectedEdge(1, 3, 8);  // School - Park: 8 min
    city.addUndirectedEdge(2, 4, 20); // Mall - Hospital: 20 min
    city.addUndirectedEdge(3, 4, 15); // Park - Hospital: 15 min
    city.addUndirectedEdge(3, 5, 25); // Park - Airport: 25 min
    city.addUndirectedEdge(4, 5, 10); // Hospital - Airport: 10 min

    fmt::print("Finding shortest times from Home to all locations:\n\n");

    auto city_result = dijkstra(city, 0);

    fmt::print("=== TRAVEL TIMES FROM HOME ===\n");
    for (size_t i = 0; i < locations.size(); i++)
    {
        if (city_result.distances[i] == INFINITE_DISTANCE)
        {
            fmt::print("{}: UNREACHABLE\n", locations[i]);
        }
        else
        {
            auto path = reconstructPath(city_result, 0, i);
            fmt::print("{}: {} minutes via ", locations[i], city_result.distances[i]);
            for (size_t j = 0; j < path.size(); j++)
            {
                fmt::print("{}", locations[path[j]]);
                if (j < path.size() - 1)
                    fmt::print(" -> ");
            }
            fmt::print("\n");
        }
    }

    return 0;
}