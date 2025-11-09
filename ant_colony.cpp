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
namespace fs = std::filesystem;
#include <climits>
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
    }
    return std::make_pair(std::move(inputs), std::move(outputs));
}

void buildPath(Ant &ant, const Graph &G,
               unordered_map<int, unordered_map<int,double>> &pheromone,
               int start, int goal, double alpha, double beta)
{
    ant.path.clear();
    ant.cost = 0.0;
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
            double tau = pow(pheromone[curr][e.to], alpha);
            int edge_weight = primes[ant.edgesVisited + 1] ? 3 * e.w2 : e.w1;
            double eta = pow(1.0 / (edge_weight + 1e-9), beta);
            double p = tau * eta;
            probs.push_back(p);
            nextNodes.push_back(e.to);
            sum += p;
        }

        if (nextNodes.empty()) break; // dead end

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
}

// ================= ACO for shortest path (single-threaded) =================
vector<int> antColonyShortestPathSparse(Graph &G,
    int start, int goal,
    int numAnts = 50, int iterations = 100,
    double alpha = 1.0, double beta = 3.0,
    double rho = 0.5, double Q = 100.0)
{
    // Sparse pheromone storage
    unordered_map<int, unordered_map<int,double>> pheromone;
    for (int u = 0; u < G.n; ++u)
        for (auto &e : G.adj[u])
            pheromone[u][e.to] = 1.0;

    vector<int> bestPath;
    double bestCost = numeric_limits<double>::infinity();

    srand(time(NULL));

    for (int iter = 0; iter < iterations; ++iter) {
        vector<Ant> ants(numAnts);

        // Build tours
        for (int i = 0; i < numAnts; ++i)
            buildPath(ants[i], G, pheromone, start, goal, alpha, beta);

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
    ios::sync_with_stdio(false);
    cin.tie(0);

    int N, M, u, v, w1, w2, start, end;
    cin >> N >> M >> start >> end;
    Graph G;
    G.n = N;
    G.adj.resize(N);

    sieve(M + 1);

    for (int i = 0; i < M; i++) {
        cin >> u >> v >> w1 >> w2;
        G.adj[u].push_back({v, w1, w2});
        G.adj[v].push_back({u, w1, w2});  // <-- 1-based index
          // <-- 1-based index
    }

    vector<int> result  = antColonyShortestPathSparse(G, start, end,N,N);

    cout << result.size() << '\n';
    for (size_t i = 0; i < result.size(); ++i)
        cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
}
