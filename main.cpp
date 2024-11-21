// COMSC-210 | Mauricio Espinosa | Lab 34
// IDE Used: Visual Studio Code & Github

#include <iostream>
#include <vector>
#include <unordered_set>
#include <stack>
#include <queue>
using namespace std;

const int SIZE = 7;

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
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // Depth First Search
    void DFS(int start) {
        unordered_set<int> visited;     // set to track visited vertices
        stack<int> stack;               // initialize the stack for the DFS
        stack.push(start);              // push the starting vertex onto the stack

        cout << "DFS starting from vertex " << start << ": " << endl;
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
                cout << vertex << " ";
                visited.insert(vertex);
            }

            // push all unvisited adjacent nodes onto the stack
            for (auto &adjacent : adjList[vertex])
            {
                if (visited.find(adjacent.first) == visited.end())
                {
                    stack.push(adjacent.first);
                }
                    
            }
        }
        cout << endl;
    }

    // Breadth First Search
    void BFS(int start) {
        unordered_set<int> visited;     // set to track visited vertices
        queue<int> queue;               // initialize the stack for the DFS
        queue.push(start);              // push the starting vertex onto the stack
        visited.insert(start);          // starting vertex is marked as visited

        cout << "BFS starting from vertex " << start << ": " << endl;
        // while the queue is not empty
        while(!queue.empty())
        {
            // assign the front value of the queue to the vertex
            int vertex = queue.front();
            // remove the front value from the queue
            queue.pop();

            // print the first vertex
            cout << vertex << " ";

            // push all unvisited adjacent nodes onto the stack
            for (auto &neighbor : adjList[vertex])
            {
                if (visited.find(neighbor.first) == visited.end())
                {
                    visited.insert(neighbor.first);
                    queue.push(neighbor.first);
                }
            }
        }
        cout << endl;
    }
};



int main() {
    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) —> edge from x to y having weight w
        {0,1,12},{0,2,8},{0,3,21},{2,3,6},{2,6,2},{5,6,6},{4,5,9},{2,4,4},{2,5,5}
    };

    // Creates graph
    Graph graph(edges);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Depth First Search
    graph.DFS(0);

    // Breadth First Search
    graph.BFS(0);

    return 0;
}
