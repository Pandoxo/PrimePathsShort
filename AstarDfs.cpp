#include <bits/stdc++.h>
using namespace std;

struct Edge {
    int to;
    int w1, w3;
};
struct Cell{
    int f;
    int node;
    int i;
};

struct Compare {
    bool operator()(Cell c1,Cell c2) {
        return c1.f > c2.f; // Min-heap: smaller values have higher priority
    }
};

//Compute prime numbers up to n using Sieve of Eratosthenes
vector<char> computePrimes(int n) {
    vector<char> isPrime(n+1, true);
    isPrime[0] = isPrime[1] = false;
    for (int i = 2; i * i <= n; i++)
        if (isPrime[i])
            for (int j = i * i; j <= n ; j += i)
                isPrime[j] = false;
    return isPrime;
}

//Compute heuristic using Dijkstra
vector<int> computeHeuristic(const int N, const vector<vector<Edge>>& adj, int target) {
    vector<int> h(N, numeric_limits<int>::max());
    using P = pair<int, int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    h[target] = 0;
    pq.push({0, target});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != h[u]) continue;
        for (auto& e : adj[u]) {
            int wmin = min(e.w1, e.w3);
            int nd = d + wmin;
            if (nd < h[e.to]) {
                h[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
    return h;
}
//Find path using A* search
vector<int> aStarSearch(int N, int start, int end,
                        vector<vector<Edge>> &adj,
                        vector<char> &primes,vector<int>& heuristic) {

    priority_queue<Cell, vector<Cell>, Compare> pq;
    vector<int> dist(N, numeric_limits<int>::max());
    vector<int> prev_node(N, -1);
    dist[start] = 0;
    pq.push({0, start, 0});

    while (!pq.empty()) {
        auto [f_u, u, i] = pq.top();
        pq.pop();

        if (f_u > dist[u] + heuristic[u]) continue;
        for (Edge e : adj[u]) {
            int v = e.to;
            int edge_cost = primes[i+1] ? e.w3 : e.w1;
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
    //Create path from start to end
    for (int v = end; v != -1; v = prev_node[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

//Compute total length of path 
int computeBlackieLength(const vector<vector<Edge>>& adj,
                        const vector<int>& path,
                        const vector<char>& isPrime) {
    int total = 0;
    for (size_t i = 1; i < path.size(); i++) {
        int u = path[i - 1], v = path[i];
        int pos = i;
        for (auto& e : adj[u]) {
            if (e.to == v) {
                total += isPrime[pos] ? e.w3 : e.w1;
                break;
            }
        }
    }
    return total;
}


void dfsImprovePath(const vector<vector<Edge>>& adj,const int start,const  int target,
                    const vector<char>& isPrime, const vector<int>& h,
                    int& bestTotal, vector<int>& bestPath) {
    int N = adj.size();
    vector<int> visited(N, 0);
    vector<int> currPath;
    auto startClock = chrono::steady_clock::now();

    function<void(int, int, int)> dfs = [&](int u, int currTotal, int depth) {
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(now - startClock).count() > 178)
            return;

        //Upper bound pruning
        if (currTotal + h[u] >= bestTotal) return;
        //Found better path
        if (u == target) {
            bestTotal = currTotal;
            bestPath = currPath;
            return;
        }

        visited[u] = 1;
        //Explore neighbors
        for (auto& e : adj[u]) {
            if (visited[e.to]) continue;
            int pos = depth + 1;
            int add = isPrime[pos] ? e.w3 : e.w1;
            currPath.push_back(e.to);
            dfs(e.to, currTotal + add, depth + 1);
            currPath.pop_back();
        }
        //Backtrack
        visited[u] = 0;
    };

    currPath.push_back(start);
    dfs(start, 0, 0);
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);

    //Input reading
    int N, M,start,target;
    cin >> N >> M >> start >> target;

    vector<vector<Edge>> adj(N);
    for (int i = 0; i < M; i++) {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        adj[u].push_back({v, w1, 3*w2});
        adj[v].push_back({u, w1, 3*w2});
    }

    //Preprocessing
    auto isPrime = computePrimes(N);
    auto h = computeHeuristic(N,adj, target);
    auto path = aStarSearch(N, start, target, adj,isPrime,h);
    
    int total = computeBlackieLength(adj, path, isPrime);
    int bestTotal = total;
    vector<int> bestPath = path;

    //Improving path using DFS
    dfsImprovePath(adj, start, target, isPrime, h, bestTotal, bestPath);

    cout << bestPath.size() << "\n";
    for (int i = 0; i < (int)bestPath.size(); i++) {
        if (i) cout << " ";
        cout << bestPath[i];
    }
}
