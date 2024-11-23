// COMSC-210 | Mauricio Espinosa | Lab 34
// IDE Used: Visual Studio Code & Github

#include <iostream>
#include <vector>
#include <string>
#include <unordered_set>
#include <stack>
#include <queue>
#include <limits>
using namespace std;

const int SIZE = 9;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Creates alias 'Pair' for the pair<int,int> data type

class Graph {
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // Graph Constructor
    Graph(vector<Edge> const &edges) {
        // resize the vector to hold SIZE elements of type vector<Edge>
        adjList.resize(SIZE);

        // add edges to the directed graph
        for (auto &edge: edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // insert at the end
            adjList[src].push_back(make_pair(dest, weight));
            // for an undirected graph, add an edge from dest to src also
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Print the graph's adjacency list
    void printGraph() {
        cout << "City Transit Network Map:\n";
        cout << "=========================\n";

        vector<string> stationNames = {
            "Central Hub", "Residential Zone", "Business District", "University", "Shopping Center",
            "Suburban Terminal", "Industrial Park", "Airport", "Stadium"
        };

        for (int i = 0; i < adjList.size(); i++) {
            cout << "Station " << i << " (" << stationNames[i] << ") connects to:\n";
            for (Pair v : adjList[i]) {
                cout << "-> Station " << v.first << " (" << stationNames[v.first] 
                << ") - Distance: " << v.second << " km\n";
            }
        }
        cout << endl;
    }

    // Depth First Search
    void DFS(int start) {
        vector<string> stationNames = {
            "Central Hub", "Residential Zone", "Business District", "University", "Shopping Center",
            "Suburban Terminal", "Industrial Park", "Airport", "Stadium"
        };

        unordered_set<int> visited;     // set to track visited vertices
        stack<int> stack;               // initialize the stack for the DFS
        stack.push(start);              // push the starting vertex onto the stack

        cout << "Transit Analysis (DFS) from Station " << start << " (" << stationNames[start] << "):\n";
        cout << "Purpose: Determining potential alternative route during service interuptions\n";
        cout << "============================================================================\n";

        // while the stack is not empty
        while(!stack.empty())
        {
            // assign the top value of the stack to the vertex
            int vertex = stack.top();
            // remove the top value from the stack
            stack.pop();

            // if the vertex has not been visited before
            if (visited.find(vertex) == visited.end())
            {
                cout << "Inspection Station " << vertex << " (" << stationNames[vertex] << " )\n";
                visited.insert(vertex);

                // push all unvisited adjacent nodes onto the stack
                for (auto &adjacent : adjList[vertex])
                {
                    if (visited.find(adjacent.first) == visited.end())
                    {
                        cout << "  -> Exploring route to Station " << adjacent.first << " ("
                        << stationNames[adjacent.first] << ") - Distance: " << adjacent.second << " km\n";

                        stack.push(adjacent.first);
                    }
                        
                }
            }
        }
        cout << endl;
    }

    // Breadth First Search
    void BFS(int start) {
        vector<string> stationNames = {
            "Central Hub", "Residential Zone", "Business District", "University", "Shopping Center",
            "Suburban Terminal", "Industrial Park", "Airport", "Stadium"
        };

        unordered_set<int> visited;     // set to track visited vertices
        queue<int> queue;               // initialize the stack for the DFS
        queue.push(start);              // push the starting vertex onto the stack
        visited.insert(start);          // starting vertex is marked as visited

        cout << "Transit Inspection (BFS) from Station " << start << " (" << stationNames[start] << ")\n";
        cout << "Purpose: Evaluating station by distance from the starting point\n";
        cout << "===============================================================\n";

        // while the queue is not empty
        while(!queue.empty())
        {
            // assign the front value of the queue to the vertex
            int vertex = queue.front();
            // remove the front value from the queue
            queue.pop();

            // print the first vertex
            cout << "Checking Station " << vertex << " (" << stationNames[vertex] << ")\n";

            // push all unvisited adjacent nodes onto the stack
            for (auto &neighbor : adjList[vertex])
            {
                if (visited.find(neighbor.first) == visited.end())
                {
                    cout << "  -> Accessible Station " << neighbor.first << " (" << stationNames[neighbor.first] 
                    << ") - Distance: " << neighbor.second << " km\n";
                    
                    visited.insert(neighbor.first);
                    queue.push(neighbor.first);
                }
            }
        }
        cout << endl;
    }

    void shortestPath(int start)
    {
        vector<string> stationNames = {
            "Central Hub", "Residential Zone", "Business District", "University", "Shopping Center",
            "Suburban Terminal", "Industrial Park", "Airport", "Stadium"
        };

        // priority queue to store distance, vertex
        priority_queue<Pair, vector<Pair>, greater<Pair>> priorityq;

        // Initialize distance of all vertices as infinity
        vector<int> dist(SIZE, numeric_limits<int>::max());

        // add source vertex to priority queue
        priorityq.push({0, start});

        // initialize distance to 0
        dist[start] = 0;

        // loop until queue is empty
        while(!priorityq.empty())
        {
            int u = priorityq.top().second;
            // remove element
            priorityq.pop();
        
            // check neighboring nodes
            for (auto &neighbor : adjList[u])
            {
                int v = neighbor.first;
                int weight = neighbor.second;
            
                // if a shorter path is found
                if (dist[u] + weight < dist[v])
                {
                    dist[v] = dist[u] + weight;
                    priorityq.push({dist[v], v});
                }
            }
        }

        cout << "Shortest path from node " << start << ":\n";
        for (int i = 0; i < SIZE; i++)
        {
            cout << start << " -> " << i << " : " << dist[i] << endl;
        }
        cout << endl;
    }

    void minimumSpanningTree() 
    {
       vector<string> stationNames = {
            "Central Hub", "Residential Zone", "Business District", "University", "Shopping Center",
            "Suburban Terminal", "Industrial Park", "Airport", "Stadium"
        }; 

        // priority queue to store distance, vertex
        priority_queue<Pair, vector<Pair>, greater<Pair>> priorityqMst;
        
        // vector to keep track of visited nodes
        vector<bool> visited(SIZE, false);

        // starting from node 0
        int startMst = 0;
        visited[startMst] = true;

        // Adding all edges beginning from the starting node
        for (auto &edge : adjList[startMst])
        {
            // push weight and destination
            priorityqMst.push({edge.second, edge.first});
        }

        cout << "Minimum Spanning Tree edges:\n";
        cout << "============================\n";

        vector<pair<int, int>> mstEdges;

        // while the queue is not empty
        while (!priorityqMst.empty())
        {
            // get smallest weight edge
            auto[weight, vertex] = priorityqMst.top();
            priorityqMst.pop();

            // skip an already visited vertex
            if(visited[vertex]) continue;

            // if not visited, then mark the vertex as visited
            visited[vertex] = true;

            // find a connecting node
            for (int u = 0; u < SIZE; u++)
            {
                for(auto &edge : adjList[u])
                {
                    if(edge.first == vertex && visited[u])
                    {
                        mstEdges.push_back({u, vertex});
                        cout << "Edge from " << u << " to " << vertex
                        << " with capacity: " << weight << " units\n";
                        break;
                    }
                }
            }

            // add unvisited adjacent edges to the queue
            for (auto &edge : adjList[vertex])
            {
                if(!visited[edge.first])
                {
                    priorityqMst.push({edge.second, edge.first});
                }
            }
        }
    }


};


int main() {
    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) —> edge from x to y having weight w
        {0,1,8},{0,2,21},{1,2,6},{1,3,5},{1,4,4},{2,7,11},{2,8,8},{3,4,9},{5,6,10},{5,7,15},{5,8,5},{6,7,3},{6,8,7}
    };

    // Creates graph
    Graph graph(edges);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Depth First Search
    graph.DFS(0);

    // Breadth First Search
    graph.BFS(0);

    // shortest path calc
    graph.shortestPath(0);

    // Minimum Spanning Tree
    graph.minimumSpanningTree();

    return 0;
}
