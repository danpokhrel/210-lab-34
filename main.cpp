// COMSC-210 | Lab 34 | Dan Pokhrel
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <limits>
using namespace std;

const int SIZE = 15;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Alias for pair<int,int>

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

    // BFS to find shortest path (by total distance)
    void findShortestPathBFS(int start, int end, unordered_map<int, string> &cityNames) {
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

            // If destination is found
            if (node == end) {
                cout << "Shortest path from " << cityNames[start] << " to " << cityNames[end]
                     << " (Total Distance: " << distance << " km): ";

                // Reconstruct the path
                vector<int> path;
                for (int at = end; at != -1; at = parent[at]) {
                    path.push_back(at);
                }
                reverse(path.begin(), path.end());

                for (int city : path) {
                    cout << cityNames[city] << " ";
                }
                cout << endl;
                return;
            }

            // Explore neighbors
            for (auto &neighbor : adjList[node]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = node;
                    q.push({neighbor.first, distance + neighbor.second});
                }
            }
        }

        cout << "No path found from " << cityNames[start] << " to " << cityNames[end] << ".\n";
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

    // Print the initial road network
    graph.printGraph(cityNames);

    // Utilize BFS for shortest path
    cout << "\nShortest Path Example (BFS):\n";
    graph.findShortestPathBFS(7, 6, cityNames); // Shortest path from CityH to CityG

    // Utilize DFS to explore all cities from a starting city
    cout << "\nExploring All Routes Example (DFS):\n";
    graph.exploreAllRoutesDFS(7, cityNames); // Explore all cities from CityH

    return 0;
}
