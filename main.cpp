// COMSC-210 | Lab 34 | Dan Pokhrel
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <limits>
#include <algorithm> // For reverse
using namespace std;

const int SIZE = 15;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Alias for pair<int, int>

// Graph class
class Graph {
public:
    vector<vector<Pair>> adjList; // Adjacency list

    Graph(vector<Edge> const &edges) {
        adjList.resize(SIZE);
        for (auto &edge : edges) {
            adjList[edge.src].push_back({edge.dest, edge.weight});
            adjList[edge.dest].push_back({edge.src, edge.weight}); // Undirected graph
        }
    }

    // Print adjacency list
    void printGraph(unordered_map<int, string> &cityNames) {
        cout << "Road Network (Adjacency List):\n";
        for (int i = 0; i < adjList.size(); i++) {
            if (!adjList[i].empty()) {
                cout << cityNames[i] << " --> ";
                for (Pair v : adjList[i]) {
                    cout << "(" << cityNames[v.first] << ", " << v.second << " km) ";
                }
                cout << endl;
            }
        }
    }

    // BFS to find paths from the starting city to all other reachable cities
    void findAllPathsBFS(int start, unordered_map<int, string> &cityNames) {
        vector<bool> visited(SIZE, false);
        queue<pair<int, int>> q; // {current city, total distance}
        unordered_map<int, int> parent; // To reconstruct the path

        // Start with the starting city
        q.push({start, 0});
        visited[start] = true;
        parent[start] = -1;

        while (!q.empty()) {
            int node = q.front().first;
            int distance = q.front().second;
            q.pop();

            // Display the path and distance to each city
            cout << "Path to " << cityNames[node] << " | Distance: " << distance << " km\n";

            // Explore neighbors
            for (auto &neighbor : adjList[node]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = node;
                    q.push({neighbor.first, distance + neighbor.second});
                }
            }
        }
    }

    // DFS to explore all cities and display distance to each
    void exploreAllRoutesDFS(int start, unordered_map<int, string> &cityNames) {
        vector<bool> visited(SIZE, false);
        stack<pair<int, int>> st; // {current city, cumulative distance}

        st.push({start, 0});

        cout << "Exploring all reachable cities from " << cityNames[start] << ":\n";

        while (!st.empty()) {
            int node = st.top().first;
            int distance = st.top().second;
            st.pop();

            if (!visited[node]) {
                cout << "City: " << cityNames[node] << " | Distance from " << cityNames[start]
                     << ": " << distance << " km\n";
                visited[node] = true;
            }

            for (auto &neighbor : adjList[node]) {
                if (!visited[neighbor.first]) {
                    st.push({neighbor.first, distance + neighbor.second});
                }
            }
        }
        cout << endl;
    }

    // Dijkstra's algorithm to find shortest paths from a single source
    void findShortestPaths(int start, unordered_map<int, string> &cityNames) {
        vector<int> distances(SIZE, numeric_limits<int>::max()); // Initialize distances
        vector<int> parents(SIZE, -1); // To reconstruct paths
        priority_queue<Pair, vector<Pair>, greater<>> minHeap; // {distance, node}

        distances[start] = 0; // Distance to itself is 0
        minHeap.push({0, start});

        while (!minHeap.empty()) {
            int node = minHeap.top().second;
            int distance = minHeap.top().first;
            minHeap.pop();

            // Skip outdated entries in the priority queue
            if (distance > distances[node]) continue;

            for (auto &neighbor : adjList[node]) {
                int nextNode = neighbor.first;
                int weight = neighbor.second;

                // Relaxation step
                if (distances[node] + weight < distances[nextNode]) {
                    distances[nextNode] = distances[node] + weight;
                    parents[nextNode] = node;
                    minHeap.push({distances[nextNode], nextNode});
                }
            }
        }

        // Print shortest paths and distances
        cout << "Shortest paths from " << cityNames[start] << ":\n";
        for (int i = 0; i < SIZE; i++) {
            if (distances[i] == numeric_limits<int>::max()) {
                cout << cityNames[start] << " -> " << cityNames[i] << ": No path\n";
            } else {
                cout << cityNames[start] << " -> " << cityNames[i] << " (Distance: " << distances[i] << " km): ";

                // Reconstruct the path
                vector<int> path;
                for (int at = i; at != -1; at = parents[at]) {
                    path.push_back(at);
                }
                reverse(path.begin(), path.end());

                for (int city : path) {
                    cout << cityNames[city] << " ";
                }
                cout << endl;
            }
        }
    }

    // Function to find Minimum Spanning Tree (MST) using Prim's Algorithm
    void findMST(int start, unordered_map<int, string> &cityNames) {
        vector<int> key(SIZE, numeric_limits<int>::max());  // Initialize all keys as infinite
        vector<int> parent(SIZE, -1); // Store the parent of each node
        vector<bool> inMST(SIZE, false); // To check if the node is included in MST
        priority_queue<Pair, vector<Pair>, greater<>> minHeap; // Min-heap for Prim's Algorithm

        key[start] = 0; // Starting city has 0 weight
        minHeap.push({0, start});

        while (!minHeap.empty()) {
            int u = minHeap.top().second;
            minHeap.pop();
            inMST[u] = true; // Include this city in MST

            // Explore neighbors
            for (auto &neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                // If the neighboring city is not in MST and has a smaller weight
                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    parent[v] = u;
                    minHeap.push({key[v], v});
                }
            }
        }

        // Print the MST
        cout << "Minimum Spanning Tree (MST):\n";
        int totalWeight = 0;
        for (int i = 0; i < SIZE; i++) {
            if (parent[i] != -1) {
                cout << cityNames[parent[i]] << " -- " << cityNames[i] << " (Distance: " << key[i] << " km)\n";
                totalWeight += key[i];
            }
        }
        cout << "Total weight of MST: " << totalWeight << " km\n";
    }
};

int main() {
    // Cities
    unordered_map<int, string> cityNames = {
        {0, "CityA"}, {1, "CityB"}, {2, "CityC"}, {3, "CityD"}, {4, "CityE"},
        {5, "CityF"}, {6, "CityG"}, {7, "CityH"}, {8, "CityI"}, {9, "CityJ"},
        {10, "CityK"}, {11, "CityL"}, {12, "CityM"}, {13, "CityN"}, {14, "CityO"}
    };

    // Initial road network
    vector<Edge> edges = {
        {7, 1, 20}, {7, 9, 30}, {7, 3, 15}, {8, 1, 25}, {8, 5, 10},
        {5, 6, 5}, {4, 5, 50}, {8, 1, 18}, {8, 5, 12}, {9, 7, 22},
        {10, 3, 35}, {11, 9, 40}, {12, 4, 45}, {13, 8, 28}, {14, 5, 60}
    };

    // Create the graph
    Graph graph(edges);

    // Main menu loop
    int choice;
    do {
        cout << "\nIntercity Road Network Menu:\n";
        cout << "[1] Display road network\n";
        cout << "[2] Trace Possible Paths from CityB (BFS)\n";
        cout << "[3] Trace Possible Paths from CityB (DFS)\n";
        cout << "[4] Calculate shortest paths from CityB\n";
        cout << "[5] Find Minimum Spanning Tree\n";
        cout << "[0] Exit\n";
        cout << "Enter your choice: ";
        cin >> choice;
        cout << endl;

        switch (choice) {
            case 1:
                graph.printGraph(cityNames);
                break;
            case 2:
                graph.findAllPathsBFS(1, cityNames); // BFS from CityB
                break;
            case 3:
                graph.exploreAllRoutesDFS(1, cityNames); // DFS from CityB
                break;
            case 4:
                graph.findShortestPaths(1, cityNames); // Shortest paths from CityB
                break;
            case 5:
                graph.findMST(1, cityNames); // MST starting from CityB
                break;
            case 0:
                cout << "Exiting the program.\n";
                break;
            default:
                cout << "Invalid choice. Please try again.\n";
        }
    } while (choice != 0);

    return 0;
}
