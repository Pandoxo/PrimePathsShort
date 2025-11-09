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
vector<int> dijkstra(Graph &G,int end) {

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
    vector<int> dist(G.n, numeric_limits<int>::infinity());

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
    return dist;
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
               const vector<int> &heuristic)
{
    ant.path.clear();
    ant.path.reserve(heuristic[start] + 10); // Pre-allocate approximate path length
    ant.cost = 0.0;
    ant.edgesVisited = 0;
    
    vector<bool> visited(G.n, false); // Faster than unordered_set for lookup
    int curr = start;
    ant.path.push_back(curr);
    visited[curr] = true;

    // Reusable buffers to avoid allocation in loop
    vector<int> nextNodes;
    vector<double> probs;
    nextNodes.reserve(G.adj[curr].size());
    probs.reserve(G.adj[curr].size());

    while (curr != goal) {
        nextNodes.clear();
        probs.clear();
        double sum = 0.0;

        for (const auto &e : G.adj[curr]) {
            if (visited[e.to]) continue;

            // Compute probability components
            double h = (heuristic[e.to] >= 0) ? 
                       1.0 / pow(heuristic[e.to] + 1.0, gamma) : 0.0;

            double tau = pow(pheromone[curr][e.to], alpha);
            int edge_weight = primes[ant.edgesVisited + 1] ? 3 * e.w2 : e.w1;
            double eta = 1.0 / (edge_weight + 1e-9);
            eta = pow(eta, beta);
            
            double p = tau * eta * h;
            
            if (p > 0.0) { // Skip zero-probability edges
                probs.push_back(p);
                nextNodes.push_back(e.to);
                sum += p;
            }
        }

        if (nextNodes.empty()) break; // Dead end

        // Roulette wheel selection (optimized)
        double r = ((double)rand() / RAND_MAX) * sum;
        double cumulative = 0.0;
        int next = nextNodes.back();
        
        for (size_t i = 0; i < nextNodes.size(); ++i) {
            cumulative += probs[i];
            if (r <= cumulative) {
                next = nextNodes[i];
                break;
            }
        }

        // Update cost
        for (const auto &e : G.adj[curr]) {
            if (e.to == next) {
                ant.edgesVisited++;
                ant.cost += (primes[ant.edgesVisited] ? 3 * e.w2 : e.w1);
                break;
            }
        }

        curr = next;
        ant.path.push_back(curr);
        visited[curr] = true;
    }
}

// Optimized ACO with better constants and strategies
vector<int> antColonyShortestPathSparse(Graph &G,
    int start, int goal,
    int numAnts = 30,           // Reduced from 50 - fewer ants, more iterations is often better
    int iterations = 200,       // Increased iterations for better convergence
    double alpha = 1.0,         // Standard pheromone influence
    double beta = 3.5,          // Strong heuristic influence for large graphs
    double gamma = 1.0,         // Strong goal direction for 100k nodes
    double rho = 0.6,           // Higher evaporation to avoid premature convergence
    double Q = 100.0,
    int elitistAnts = 3)        // Number of best ants that deposit extra pheromone
{
    // Sparse pheromone storage with initial value
    unordered_map<int, unordered_map<int,double>> pheromone;
    const double initialPheromone = 1.0;
    
    for (int u = 0; u < G.n; ++u) {
        for (const auto &e : G.adj[u]) {
            pheromone[u][e.to] = initialPheromone;
        }
    }

    vector<int> bestPath;
    vector<int> heuristic = dijkstra(G, goal);
    double bestCost = numeric_limits<double>::infinity();
    int stagnationCounter = 0;
    const int maxStagnation = 20; // Early stopping if no improvement

    srand(time(NULL));

    for (int iter = 0; iter < iterations; ++iter) {
        vector<Ant> ants(numAnts);

        // Build tours
        for (int i = 0; i < numAnts; ++i) {
            buildPath(ants[i], G, pheromone, start, goal, alpha, beta, gamma, heuristic);
        }

        // Sort ants by cost (for elitist strategy)
        vector<Ant*> validAnts;
        for (auto &ant : ants) {
            if (!ant.path.empty() && ant.path.back() == goal && ant.cost > 0) {
                validAnts.push_back(&ant);
            }
        }
        
        sort(validAnts.begin(), validAnts.end(), 
             [](const Ant* a, const Ant* b) { return a->cost < b->cost; });

        // Reset pheromone delta
        unordered_map<int, unordered_map<int,double>> delta;

        // Deposit pheromone (elitist strategy - best ants deposit more)
        for (size_t idx = 0; idx < validAnts.size(); ++idx) {
            Ant* ant = validAnts[idx];
            
            // Update best solution
            if (ant->cost < bestCost) {
                bestCost = ant->cost;
                bestPath = ant->path;
                stagnationCounter = 0;
                cout << "*** New best found at iteration " << iter+1 
                     << " | Cost: " << bestCost << " ***\n";
            }
            
            // Elitist ants deposit more pheromone
            double depositAmount = Q / ant->cost;
            if (idx < (size_t)elitistAnts) {
                depositAmount *= 2.0; // Double pheromone for elite ants
            }
            
            for (size_t i = 0; i + 1 < ant->path.size(); ++i) {
                int a = ant->path[i], b = ant->path[i+1];
                delta[a][b] += depositAmount;
                delta[b][a] += depositAmount;
            }
        }

        // Additional deposit for global best (intensification)
        if (!bestPath.empty() && iter % 5 == 0) { // Every 5 iterations
            double bestDeposit = Q / bestCost * 0.5;
            for (size_t i = 0; i + 1 < bestPath.size(); ++i) {
                int a = bestPath[i], b = bestPath[i+1];
                delta[a][b] += bestDeposit;
                delta[b][a] += bestDeposit;
            }
        }

        // Evaporation + add new pheromone
        const double minPheromone = 0.01;  // Prevent pheromone from becoming too small
        const double maxPheromone = 10.0;   // Cap maximum pheromone
        
        for (auto &[u, mp] : pheromone) {
            for (auto &[v, p] : mp) {
                double add = delta[u][v];
                p = (1.0 - rho) * p + add;
                // Clamp pheromone values
                p = max(minPheromone, min(maxPheromone, p));
            }
        }

        stagnationCounter++;
        
        // Early stopping if stagnated
        // if (stagnationCounter >= maxStagnation) {
        //     cout << "Stagnation detected. Stopping early at iteration " << iter+1 << "\n";
        //     break;
        // }

        if ((iter + 1) % 10 == 0 || iter == 0) {
            cout << "Iteration " << iter+1 << " | Best cost: " << bestCost 
                 << " | Valid ants: " << validAnts.size() << "/" << numAnts << "\n";
        }
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
        vector<int> result  = antColonyShortestPathSparse(G, start, end,100,100);
        cout << result.size() << '\n';
        // for (size_t i = 0; i < result.size(); ++i){
        //     cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
        // }
    }
    return 0;
}