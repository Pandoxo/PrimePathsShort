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

struct Edge {
    int to, w1, w2, idx;
};

struct Graph { 
    int n;
    vector<vector<Edge>> adj;
};

struct Ant {
     vector<int> path;
      double cost = 0.0;
      int edgesVisited = 0;
     };

double rand01() { return (double)rand() / RAND_MAX; }


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



auto load_test_cases() {
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
pair<vector<int>,vector<int>> dijkstra(Graph &G,int start,int end) {

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
    vector<int> dist(G.n, numeric_limits<int>::infinity());
    vector<int> parent(G.n, -1);
    dist[end] = 0;
    pq.push({0, end});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        for (Edge e : G.adj[u]) {
            int cost = min(3*e.w2,e.w1);
            if (dist[u] + cost < dist[e.to]) {
                dist[e.to] = dist[u] + cost;
                pq.push({dist[e.to], e.to});
            }
        }
    }
    vector<int> path;
    int curr = start;
    while(curr!= -1){
        path.push_back(curr);
        curr = parent[curr];
    }
    return {dist,path};
}

vector<int> bfsDistance(const Graph &G, int goal) {
    cout<<"Computing BFS distances to goal node "<<goal<<"\n";
    vector<int> dist(G.n, -1);
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
    return dist;
}

void buildPath(Ant &ant, const Graph &G,
               unordered_map<int, unordered_map<int,double>> &pheromone,
               int start, int goal, double alpha, double beta, double gamma,
               vector<int> &heuristic, int maxRestarts = 3)
{
    for (int restart = 0; restart <= maxRestarts; ++restart) {
        ant.path.clear();
        ant.cost = 0.0;
        ant.edgesVisited = 0;
        unordered_set<int> visited;
        int curr = start;
        ant.path.push_back(curr);
        visited.insert(curr);

        while (curr != goal) {
            vector<int> nextNodes;
            vector<double> probs;
            double sum = 0.0;

            for (auto &e : G.adj[curr]) {
                if (visited.count(e.to)) continue;
                
                double h = (heuristic[e.to] >= 0) ? 
                           pow(1.0 / (heuristic[e.to] + 1.0), gamma) : 0.0;

                double tau = pow(pheromone[curr][e.to], alpha);
                int  edge_weight = primes[ant.edgesVisited + 1] ? 3 * e.w2 : e.w1;
                double eta = pow(1.0 / (edge_weight + 1e-9), beta);
                double p = tau * eta * h;
                probs.push_back(p);
                nextNodes.push_back(e.to);
                sum += p;
            }

            if (nextNodes.empty()) break; // dead end - will restart

            double r = rand01() * sum;
            double cumulative = 0.0;
            int next = nextNodes.back();
            for (int i = 0; i < (int)nextNodes.size(); ++i) {
                cumulative += probs[i];
                if (r <= cumulative) { next = nextNodes[i]; break; }
            }

            for (auto &e : G.adj[curr]) {
                if (e.to == next) {
                    ant.edgesVisited++;
                    ant.cost += (primes[ant.edgesVisited] ? 3 * e.w2 : e.w1);
                    break;
                }
            }

            curr = next;
            ant.path.push_back(curr);
            visited.insert(curr);
        }
        
        // If reached goal, done!
        if (curr == goal) break;
    }
}
// ================= ACO for shortest path (single-threaded) =================
vector<int> antColonyShortestPathSparse(Graph &G,
    int start, int goal,
    int numAnts = 50,
    int iterations = 100,
    double alpha = 0.9, 
    double beta = 3.0, 
    double gamma =   3.0,
    double rho = 0.7, 
    double Q = 100.0)
{
    // Sparse pheromone storage
    unordered_map<int, unordered_map<int,double>> pheromone;
    for (int u = 0; u < G.n; ++u) {
        for (auto &e : G.adj[u]){
            pheromone[u][e.to] = 1.0;
            pheromone[e.to][u] = 1.0;
        }
    }
    
    vector<int> bestPath;
    //vector<int> heuristic = bfsDistance(G, goal);
    auto [heuristic,dijkstraPath] = dijkstra(G, start,goal);

    if (!dijkstraPath.empty() && dijkstraPath.back() == goal) {
        for (int i = 0; i + 1 < (int)dijkstraPath.size(); ++i) {
            int a = dijkstraPath[i], b = dijkstraPath[i+1];
            pheromone[a][b] = 13.0;  // Strong initial signal
            pheromone[b][a] = 13.0;
        }
    }

    double bestCost = numeric_limits<double>::infinity();

    srand(time(NULL));

    for (int iter = 0; iter < iterations; ++iter) {
        vector<Ant> ants(numAnts);

        // Build tours
        for (int i = 0; i < numAnts; ++i)
            buildPath(ants[i], G, pheromone, start, goal, alpha, beta, gamma,heuristic);

        // Reset pheromone delta
        unordered_map<int, unordered_map<int,double>> delta;

        // Deposit pheromone
        for (auto &ant : ants) {
            if (!ant.path.empty() && ant.path.back() == goal && ant.cost > 0) {
                if (ant.cost < bestCost) {
                    bestCost = ant.cost;
                    bestPath = ant.path;
                }
                for (int i = 0; i + 1 < (int)ant.path.size(); ++i) {
                    int a = ant.path[i], b = ant.path[i+1];
                    delta[a][b] += Q / ant.cost;
                    delta[b][a] += Q / ant.cost;
                }
            }
        }

        // Evaporation + add new pheromone
        for (auto &[u, mp] : pheromone) {
            for (auto &[v, p] : mp) {
                double add = delta[u][v];
                p = (1 - rho) * p + add;
            }
        }

        cout << "Iteration " << iter+1 << " | Best cost: " << bestCost << "\n";
    }

    return bestPath;
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
        for (int i = 0; i < M; i++) {
            (*f) >> u >> v >> w1 >> w2;
            G.adj[u].push_back({v, w1, w2, i + 1});
            G.adj[v].push_back({u, w1, w2, i + 1});
             // <-- 1-based index
        }
        vector<int> result  = antColonyShortestPathSparse(G, start, end,2000,10);
        cout << result.size() << '\n';
        // for (size_t i = 0; i < result.size(); ++i){
        //     cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
        // }
    }
    return 0;
}