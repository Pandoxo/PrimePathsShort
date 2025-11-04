#include <bits/stdc++.h>
using namespace std;

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
                     vector<vector<tuple<int,int,int,int>>> &adj,
                     vector<bool> &primes) {
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
    vector<int> dist(N, INT_MAX), prev_node(N, -1);

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != dist[u]) continue;

        for (auto &[v, w1, w2, i] : adj[u]) {
            int edgeCost = primes[i] ? 3 * w2 : w1;
            if (dist[u] + edgeCost < dist[v]) {
                dist[v] = dist[u] + edgeCost;
                prev_node[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[end] == INT_MAX) return {}; // no path

    // reconstruct path
    vector<int> path;
    for (int v = end; v != -1; v = prev_node[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int N, M, u, v, w1, w2, start, end;
    cin >> N >> M >> start >> end;

    vector<bool> primes = sieve(M);
    vector<vector<tuple<int,int,int,int>>> adj(N);

    for (int i = 0; i < M; i++) {
        cin >> u >> v >> w1 >> w2;
        adj[u].push_back({v, w1, w2, i});
    }

    vector<int> result = dijkstra(N, start, end, adj, primes);

    cout << result.size() << '\n';
    for (size_t i = 0; i < result.size(); ++i)
        cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
}
