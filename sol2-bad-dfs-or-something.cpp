#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <fstream>
#include <filesystem>
#include <chrono>
namespace fs = std::filesystem;
#include <climits>
using namespace std;

//time
using namespace std::chrono;
const int TIME_LIMIT_SECONDS = 10;
steady_clock::time_point start_time;
auto now = steady_clock::now();//nwm
//end of time

struct Edge{
    int to, w1, w2;
};

struct Cell{ //for finding heuristic
    long long distance;
    int node;
};

struct node{ //for using A*
    int node;
    int distance;
    int heuristic;
    int i;
};

struct Compare {
    bool operator()(Cell c1,Cell c2) {
        return c1.distance > c2.distance; // Min-heap: smaller values have higher priority
    }
};

struct CompareHeuristic {
    bool operator()(node n1,node n2) {
        return n1.distance + n1.heuristic> n2.distance + n2.heuristic; // Min-heap: smaller values have higher priority
    }
};

vector<bool> sieve(int n) {
    vector<bool> primes(n + 1, true);
    primes[0] = primes[1] = false;
    for (int p = 2; p * p <= n; p++)
    {
        if (primes[p])
        {
            for (int i = p * p; i <= n; i += p)
                primes[i] = false;
        }
    }
    return primes;
}

int evaluate_sol(vector<int> &sol, vector<vector<Edge>> &adj, vector<bool> &primes)
{
    int value = 0;
    int size = sol.size();
    for(int i = 0; i<size-1;++i)
    {
        int from = sol[i];
        int next = sol[i+1];
        for(int j = 0; j<adj[from].size();++j)
        {
            if (adj[from][j].to == next)
            {
                primes[i+1] ? value+= adj[from][j].w1 : value += adj[from][j].w2;
                break;
            }
        }
    }
    return value;
}

vector<int> find_heuristic(int N, int start, int end, vector<vector<Edge>> &adj)
{

    priority_queue<Cell, vector<Cell>, Compare> pq;
    vector<int> dist(N, INT_MAX);

    dist[end] = 0;
    pq.push({0, end});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        for (Edge e : adj[u])
        {
            int cost = min(e.w2,e.w1);
            if (dist[u] + cost < dist[e.to])
            {
                dist[e.to] = dist[u] + cost;
                pq.push({dist[e.to], e.to});
            }
        }
    }
    return dist;
}

vector<int> Astar(int N, int start, int end,
vector<vector<Edge>> &adj,vector<bool> &primes, vector<int> &heuristic)
{
    priority_queue<node, vector<node>, CompareHeuristic> pq;

    vector<int> dist(N, INT_MAX);
    vector<int> prev_node(N, -1);

    dist[start] = 0;
    pq.push({start,0,heuristic[start],0});

    while (!pq.empty()) {
        //auto [d, u, i] = pq.top();
        auto [node,d,h,i] = pq.top();
        pq.pop();
        if (d > dist[node] or i>N) continue;
        for (Edge e : adj[node])
        {
            int cost = primes[i+1] ? e.w2 : e.w1;
            if (dist[node] + cost < dist[e.to])
            {
                dist[e.to] = dist[node] + cost;
                prev_node[e.to] = node;
                pq.push({e.to,dist[e.to], heuristic[e.to],i+1});
            }
        }
    }

    if (dist[end] == INT_MAX) return {};

    vector<int> path;
    for (int v = end; v != -1; v = prev_node[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

void dfs_bound(int &global_bound,int count, int i, int node, int end,
    vector<vector<Edge>> &adj, vector<int> &dfs_bond_sol, vector<int> &best_sol, vector<bool> &primes)
{   
    if (count > global_bound) return;
    if(node == end)
    {
        best_sol = dfs_bond_sol;
        global_bound = count;
        return;
    }
    now = steady_clock::now();
    auto elapsed = duration_cast<seconds>(now - start_time).count();
    if (elapsed >= TIME_LIMIT_SECONDS) return;

    for(Edge e : adj[node])
    {
        int w;
        primes[i] == true ? w=e.w1 : w=e.w2;
        dfs_bond_sol.push_back(e.to);
        dfs_bound(global_bound, count+w, i+1, e.to, end, adj, dfs_bond_sol, best_sol, primes);
        dfs_bond_sol.pop_back();

        now = steady_clock::now();
        elapsed = duration_cast<seconds>(now - start_time).count();
        if (elapsed >= TIME_LIMIT_SECONDS) return;
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


int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);
    int N, M, u, v, w1, w2, start, end;
    start_time = steady_clock::now();
    
    //auto [inputs,outputs] = load_test_cases();//
    //int test_case = 0;//
    //for (auto& f : inputs)//
    //{//
        //cout << "Test case " << test_case++ << ":\n";//
        //(*f) >> N >> M >> start >> end;//
        cin >> N >> M >> start >> end;//

        vector<bool> primes = sieve(M + 1);
        vector<vector<Edge>> adj(N);

        for (int i = 0; i < M; i++)
        {
            //(*f) >> u >> v >> w1 >> w2;//
            cin >> u >> v >> w1 >> w2;//

            adj[u].push_back({v, w1, 3*w2});
            adj[v].push_back({u, w1, 3*w2});
        }
        vector<int> heuristic = find_heuristic(N, start, end, adj);

        vector<int> result = Astar(N, start, end, adj, primes, heuristic);
        cout << result.size() << '\n';
        int global_bound = evaluate_sol(result, adj, primes);

        vector<int> dfs_bond_sol;
        dfs_bond_sol.push_back(start);
        dfs_bound(global_bound, 0, 1, start, end, adj, dfs_bond_sol, result, primes);
        //cout<<global_bound;

        cout<<result.size()<<"\n";
        for (size_t i = 0; i < result.size(); ++i)
        {
           cout << result[i] << (i + 1 == result.size() ? '\n' : ' ');
        }
    //}//
    return 0;
}