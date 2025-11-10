#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <math.h>
#include <climits>
using namespace std;

struct Edge {
    int to, w1, w2;
};
struct Cell{
    long long distance;
    int node;
    int i;
};
struct NodeState {
    int v;
    int dist;
    int depth; // how many edges used so far
    bool operator>(const NodeState &o) const { return dist > o.dist; }
};

struct Compare {
    bool operator()(Cell c1,Cell c2) {
        return c1.distance > c2.distance; // Min-heap: smaller values have higher priority
    }
};

vector<bool> sieve(int n) {
    vector<bool> primes(n + 1, true);
    primes[0] = primes[1] = false;
    for (int p = 2; p * p <= n; p++) {
        if (primes[p]) {
            for (int i = p * p; i <= n; i += p)
                primes[i] = false;
        }
    }
    return primes;
}
vector<int> dijkstra(int N, int start, int end,
                     vector<vector<Edge>> &adj,
                     vector<bool> &primes) {

    priority_queue<Cell, vector<Cell>, Compare> pq;
    vector<int> dist(N, INT_MAX), prev_node(N, -1);

    dist[start] = 0;
    pq.push({0, start,0});

    while (!pq.empty()) {
        auto [d, u, i] = pq.top();
        pq.pop();
        if (d > dist[u] or i>N or d > dist[end]) continue;
        for (Edge e : adj[u]) {
            int cost = min(3*e.w2,e.w1);
            if (dist[u] + cost < dist[e.to]) {
                dist[e.to] = dist[u] + cost;
                prev_node[e.to] = u;
                pq.push({dist[e.to], e.to,i+1});
            }
        }
    }
    return dist;
}
vector<int> limitedDijkstra(const vector<vector<Edge>> &adj,
                            int src, int dst,
                            int startEdgeIndex, int maxDepth,
                            const vector<bool> &primes,
                            int costLimit = INT_MAX,
                            int nodeLimit = 5000)
{
    const int n = adj.size();
    vector<int> dist(n, INT_MAX), parent(n, -1), depth(n, -1);
    priority_queue<NodeState, vector<NodeState>, greater<NodeState>> pq;

    dist[src] = 0;
    depth[src] = 0;
    pq.push({src, 0, 0});
    int explored = 0;

    while (!pq.empty() && explored < nodeLimit) {
        auto [v, d, steps] = pq.top();
        pq.pop();
        explored++;

        if (d != dist[v]) continue;
        if (d >= costLimit) break;
        if (v == dst) break;
        if (steps >= maxDepth) continue;

        for (auto &e : adj[v]) {
            int nextStep = steps + 1;
            int edgeIdx = startEdgeIndex + nextStep - 1; // global edge index (1-indexed)
            int w = (primes[edgeIdx] ? e.w2 : e.w1);
            int nd = d + w;

            if (nd < dist[e.to] && nd < costLimit) {
                dist[e.to] = nd;
                parent[e.to] = v;
                depth[e.to] = nextStep;
                pq.push({e.to, nd, nextStep});
            }
        }
    }

    if (dist[dst] == INT_MAX) return {}; // no better path found

    // reconstruct path
    vector<int> path;
    for (int v = dst; v != -1; v = parent[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

int subpathCost(const vector<vector<Edge>> &adj,
                const vector<int> &path,
                int i, int j,
                const vector<bool> &primes)
{
    int total = 0;
    for (int k = i; k < j; k++) {
        int u = path[k], v = path[k + 1];
        bool found = false;
        for (auto &e : adj[u]) {
            if (e.to == v) {
                int edgeIdx = k + 1; // global edge index (1-indexed)
                total += primes[edgeIdx] ? e.w2 : e.w1;
                found = true;
                break;
            }
        }
        if (!found) return INT_MAX;
    }
    return total;
}

void replaceSubpath(vector<int> &path, int i, int j, const vector<int> &newSeg) {
    vector<int> newPath;
    newPath.reserve(path.size() - (j - i + 1) + newSeg.size());
    newPath.insert(newPath.end(), path.begin(), path.begin() + i);
    newPath.insert(newPath.end(), newSeg.begin(), newSeg.end());
    newPath.insert(newPath.end(), path.begin() + j + 1, path.end());
    path.swap(newPath);
}

// ---- One-pass local refinement ----
void refinePathSinglePass(vector<vector<Edge>> &adj, vector<int> &path,
                          const vector<bool> &primes, int WINDOW = 15)
{
    for (int i = 0; i < (int)path.size(); i++) {
        for (int j = i + 3; j < (int)path.size() && j <= i + WINDOW; j++) {
            int currentCost = subpathCost(adj, path, i, j, primes);
            if (currentCost == INT_MAX) continue;

            int limit = currentCost - 1;
            auto newSeg = limitedDijkstra(adj, path[i], path[j],
                                          i + 1, j - i, primes, limit);
            if (!newSeg.empty()) {
                int newCost = subpathCost(adj, newSeg, 0, (int)newSeg.size() - 1, primes);
                if (newCost < currentCost) {
                    replaceSubpath(path, i, j, newSeg);
                    // Skip ahead to avoid reusing overlapping parts in same pass
                    i = max(i - 2, 0);
                    break;
                }
            }
        }
    }
}


vector<int> aStarSearch(int N, int start, int end,
                        vector<vector<Edge>> &adj,
                        vector<bool> &primes) {

    priority_queue<Cell, vector<Cell>, Compare> pq;
    vector<int> dist(N, INT_MAX);
    vector<int> prev_node(N, -1);
    vector<int> heuristic = dijkstra(N,end,start,adj,primes); // Heuristic: shortest path from node to end
    dist[start] = 0;;
    pq.push({0, start, 0});

    while (!pq.empty()) {
        auto [f_u, u, i] = pq.top();
        pq.pop();

        if (f_u > dist[u] + heuristic[u]) continue;
        if (u == end) {
            break;
        }
        for (Edge e : adj[u]) {
            int v = e.to;
            int edge_cost =  sqrt(min(e.w2, e.w1)); // Use the min weight as cost
            int new_g_v = dist[u] + edge_cost;
            if (new_g_v < dist[v]) {
                dist[v] = new_g_v;
                prev_node[v] = u;
                int new_f_v = new_g_v + heuristic[v];
                pq.push({new_f_v, v, i + 1});
            }
        }
    }
    vector<int> path;
    for (int v = end; v != -1; v = prev_node[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}


// int path_cost(const vector<int>& path, const vector<vector<Edge>>& adj,vector<bool>& primes) {

//     int total_cost = 0;
//     for (size_t i = 1; i< path.size(); ++i) {
//         auto edge = find_if(adj[path[i-1]].begin(), adj[path[i-1]].end(),
//                             [&](const Edge& e) { return e.to == path[i]; });
//         if (edge != adj[path[i-1]].end()) {
//             total_cost += (primes[i]) ? 3 * edge->w2 : edge->w1;
//         }
//     }
//     return total_cost;
// }
int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);

    int N, M, u, v, w1, w2, start, end;
    cin >> N >> M >> start >> end;

    vector<bool> primes = sieve(M + 1);
    vector<vector<Edge>> adj(N);

    for (int i = 0; i < M; i++) {
        cin >> u >> v >> w1 >> w2;
        adj[u].push_back({v, w1, w2});
        adj[v].push_back({u, w1, w2});  // <-- 1-based index
          // <-- 1-based index
    }

    vector<int> result = aStarSearch(N, start, end, adj, primes);
    refinePathSinglePass(adj, result, primes, 10);
    cout << result.size() << '\n';
    for (size_t i = 0; i < result.size(); ++i)
        cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
}