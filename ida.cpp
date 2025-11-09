#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <ctime>
#include <math.h>
#include <filesystem>
#include <climits>


namespace fs = std::filesystem;
using namespace std;


vector<bool> primes;
vector<int> heuristic;
struct Edge {
    int to, w1, w2;
};


void sieve(int n) {
    primes.resize(n + 1, true);
    primes[0] = primes[1] = false;
    for (int p = 2; p * p <= n; p++) {
        if (primes[p]) {
            for (int i = p * p; i <= n; i += p)
                primes[i] = false;
        }
    }
    cout<<"Sieve done up to "<<n<<"\n";
}



struct Graph { 
    int n;
    vector<vector<Edge>> adj;
};

// Heuristic function (can be customized based on your needs)
vector<int> bfsDistance(const Graph &G, int goal) {
    vector<int> dist(G.n,-1);
    queue<int> q;
    q.push(goal);
    dist[goal] = 0;
    
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (const auto& e : G.adj[u]) {
            if (dist[e.to] == -1) {
                dist[e.to] = dist[u] + 1;
                q.push(e.to);
            }
        }
    }
    cout<<"Computed BFS distances\n";
    return dist;
}

// Recursive depth-limited search
int ida_search(const Graph& g, vector<int>& path, int g_cost, int bound, 
               int goal, vector<bool>& visited) {
    int node = path.back();
    int f = g_cost + heuristic[node];
    
    if (f > bound) {
        return f;
    }
    
    if (node == goal) {
        std::cout<<"Goal found with cost "<<g_cost<<"\n";
        return -1;
    }
    
    int min_bound = INT_MAX;
    
    for (const Edge& e : g.adj[node]) {
        if (!visited[e.to]) {
            visited[e.to] = true;
            path.push_back(e.to);
            
            int edge_cost = (primes[path.size()-1])? 3*e.w2 : e.w1;
            int t = ida_search(g, path, g_cost + edge_cost, bound, goal, visited);
            
            if (t == -1) {
                return -1;
            }
            if (t < min_bound) {
                min_bound = t;
            }
            
            path.pop_back();
            visited[e.to] = false;
        }
    }
    
    return min_bound;
}

bool ida_star(const Graph& g, int start, int end, vector<int>& path) {
    path.clear();
    
    int bound = heuristic[start];
    int iteration = 0;
    
    while (true) {
        vector<int> current_path = {start};  // Reset each iteration
        vector<bool> visited(g.n, false);     // Reset each iteration
        visited[start] = true;
        
        
        int t = ida_search(g, current_path, 0, bound, end, visited);
        cout<<"Iteration: "<< iteration++
            << " Current bound: "<<bound<<"\n";
        
        if (t == -1) {
            path = current_path;
            return true;
        }
        if (t == INT_MAX) {
            return false;
        }
        bound = t;
    }
}
auto load_test_cases(){
    fs::path srcPath = __FILE__;
    fs::path srcDir = srcPath.parent_path();
    fs::path targetFolder = srcDir / "co-project1-ex-input";

    std::vector<fs::path> filePaths;
    for (const auto& entry : fs::directory_iterator(targetFolder))
        if (entry.is_regular_file())
            filePaths.push_back(entry.path());

    std::sort(filePaths.begin(), filePaths.end(),
        [](const fs::path& a, const fs::path& b) {
            return a.filename().string() < b.filename().string();
        });

    std::vector<std::unique_ptr<std::ifstream>> inputs;
    std::vector<std::unique_ptr<std::ifstream>> outputs;

    int i = 0;
    for (const auto& p : filePaths) {
        auto f = std::make_unique<std::ifstream>(p);
        if (!f->is_open()) {
            std::cerr << "Failed to open " << p << "\n";
            continue;
        }
        if (i % 2 == 0)
            inputs.push_back(std::move(f));
        else
            outputs.push_back(std::move(f));
        i++;
        cout<<"Loaded "<<p<<"\n";
    }
    return std::make_pair(std::move(inputs), std::move(outputs));
}


int main() {
    cout<<"Starting tests...\n";

    int N, M, u, v, w1, w2, start, end;
    cout<<"Generating sieve...\n";
    cout<< "Loading test cases...\n";
    auto [inputs,outputs] = load_test_cases();
    int test_case = 0;
    for (auto& f : inputs) {
        cout << "Test case " << test_case++ << ":\n";
        (*f) >> N >> M >> start >> end;
        sieve(M);
        Graph G;
        G.n = N;
        G.adj.resize(N);
        vector<int> result;
        for (int i = 0; i < M; i++) {
            (*f) >> u >> v >> w1 >> w2;
            G.adj[u].push_back({v, w1, w2});
            G.adj[v].push_back({u, w1, w2});
        }
        heuristic = bfsDistance(G, end);
        ida_star(G,start, end,result);
        cout << result.size() << '\n';
        // for (size_t i = 0; i < result.size(); ++i){
        //     cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
        // }
    }
    return 0;
}


