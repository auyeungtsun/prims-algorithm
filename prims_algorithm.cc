#include <iostream>
#include <vector>
#include <queue>
#include <limits>

#include <cassert>
using namespace std;

/**
 * @brief Computes the Minimum Spanning Tree (MST) cost using Prim's algorithm.
 * @brief Minimum Spanning Tree (MST)
 * In graph theory, a minimum spanning tree (MST) or minimum weight spanning tree is a subset of the edges 
 * of a connected, edge-weighted undirected graph that connects all the vertices together, without any cycles and with the minimum possible total edge weight. 
 * 
 * @param n The number of vertices in the graph.
 * @param adj The adjacency list representing the graph. Each element adj[i] is a vector 
 *            of pairs, where each pair (v, weight) represents an edge from vertex i to 
 *            vertex v with the given weight.
 * @return The total cost of the Minimum Spanning Tree.
 * 
 * @details This function implements Prim's algorithm to find the MST of a connected, 
 *          undirected, weighted graph. It starts from an arbitrary vertex (vertex 0 in 
 *          this case) and iteratively adds the minimum-weight edge that connects a 
 *          vertex in the MST to a vertex not yet in the MST.
 * @note Time Complexity: O(E log V), where E is the number of edges and V is the number of vertices.
 * @note Space Complexity: O(V), where V is the number of vertices.
 */
int primMST(int n, const vector<vector<pair<int, int>>>& adj) {
    // pq is a priority queue used to store edges that are candidates for inclusion in the MST.
    // Each element in the priority queue is a pair<int, int>, where:
    //   - The first element (int) is the weight of the edge.
    //   - The second element (int) is the destination vertex of the edge.
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    // key[i] is min weight to connect i node to MST
    vector<int> key(n, numeric_limits<int>::max());
    // inMST[i] is true if vertex i is already included in the MST, false otherwise.
    vector<bool> inMST(n, false);
    // mstCost keeps track of the total weight of the MST.

    int mstCost = 0;

    pq.push({0, 0});
    key[0] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (inMST[u]) continue;
        inMST[u] = true;
        mstCost += key[u];

        for (auto& edge : adj[u]) {
            int v = edge.first;
            int weight = edge.second;
            if (!inMST[v] && weight < key[v]) {
                key[v] = weight;
                pq.push({key[v], v});
            }
        }
    }
    return mstCost;
}

void testPrimMST() {
    // Test case 1: Simple graph
    int n1 = 4;
    vector<vector<pair<int, int>>> adj1(n1);
    adj1[0].push_back({1, 10});
    adj1[0].push_back({2, 6});
    adj1[0].push_back({3, 5});
    adj1[1].push_back({0, 10});
    adj1[1].push_back({3, 15});
    adj1[2].push_back({0, 6});
    adj1[2].push_back({3, 4});
    adj1[3].push_back({0, 5});
    adj1[3].push_back({1, 15});
    adj1[3].push_back({2, 4});
    assert(primMST(n1, adj1) == 19);

    // Test case 2: Larger graph
    int n2 = 5;
    vector<vector<pair<int, int>>> adj2(n2);
    adj2[0].push_back({1, 2});
    adj2[0].push_back({3, 6});
    adj2[1].push_back({0, 2});
    adj2[1].push_back({2, 3});
    adj2[1].push_back({3, 8});
    adj2[1].push_back({4, 5});
    adj2[2].push_back({1, 3});
    adj2[2].push_back({4, 7});
    adj2[3].push_back({0, 6});
    adj2[3].push_back({1, 8});
    adj2[3].push_back({4, 9});
    adj2[4].push_back({1, 5});
    adj2[4].push_back({2, 7});
    adj2[4].push_back({3, 9});
    assert(primMST(n2, adj2) == 16);

    // Test case 3: Single node graph
    int n3 = 1;
    vector<vector<pair<int, int>>> adj3(n3);
    assert(primMST(n3, adj3) == 0);
}

void runPrimMSTSample() {
    int n = 5;
    vector<vector<pair<int, int>>> adj(n);
    adj[0].push_back({1, 2});
    adj[0].push_back({3, 6});
    adj[1].push_back({0, 2});
    adj[1].push_back({2, 3});
    adj[1].push_back({3, 8});
    adj[1].push_back({4, 5});
    adj[2].push_back({1, 3});
    adj[2].push_back({4, 7});
    adj[3].push_back({0, 6});
    adj[3].push_back({1, 8});
    adj[3].push_back({4, 9});
    adj[4].push_back({1, 5});
    adj[4].push_back({2, 7});
    adj[4].push_back({3, 9});
    cout << "MST cost: " << primMST(n, adj) << endl;
}

int main() {
    testPrimMST();
    runPrimMSTSample();
    return 0;
}

