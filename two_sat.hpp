template<class T = int>
struct Edge {
	int u, v;
	T w;
	Edge() = default;
	Edge(int u, int v, int w = 1) : u(u), v(v), w(w) {
	operator int() const { return v; }
	friend ostream &operator << (ostream &os, const Edge &e) {
		return os << "(" << e.u << " -> " << e.v << "): " << e.w;
	}
};
template<class T>
using Edges = vector<Edge<T>>;
template<class T = int>
struct Graph {
	vector<Edges<T>> g;
	Graph() = default;
	Graph(int n) : g(n + 1) {}
	void addEdge(int u, int v, T w = 1) {
		g[u].emplace_back(u, v, w);
	}
	inline Edges<T> &operator [] (const int &u) {
		return g[u];
	}
	inline const Edges<T> &operator [] (const int &u) const {
		return g[u];
	}
};
struct SCC {
	int cnt;
	vector<int> dfn, low, idx, siz;
	vector<bool> vis;
	SCC() = default;
	template<class Graph>
	SCC(int n, const Graph &G) : 
		cnt(0), dfn(n + 1), low(n + 1), idx(n + 1), siz(n + 1), vis(n + 1) {
		int now = 0;
		stack<int, vector<int>> stk;
		function<void(int)> dfs = [&](int u) {
			dfn[u] = low[u] = ++now;
			stk.push(u);
			vis[u] = true;
			for (int v : G[u]) {
				if (!dfn[v]) {
					dfs(v);
					low[u] = min(low[u], low[v]);
				} else if (vis[v]) {
					low[u] = min(low[u], low[v]);
				}
			}
			if (dfn[u] == low[u]) {
				idx[u] = ++cnt;
				siz[cnt] = 1;
				vis[u] = false;
				while (stk.top() != u) {
					idx[stk.top()] = cnt;
					vis[stk.top()] = false;
					siz[cnt] += 1;
					stk.pop();
				}
				stk.pop();
			}
		};
		for (int i = 1; i <= n; i++) {
			if (!dfn[i]) dfs(i);
		}
	}
	int count() {
		return cnt;
	}
	int size(int i) {
		return siz[i];
	}
	int find(int u) {
		return idx[u];
	}
	bool same(int u, int v) {
		return idx[u] == idx[v];
	}
};
struct TwoSat {
	int n;
	vector<vector<int>> g;
	TwoSat() = default;
	TwoSat(int n) : n(n), g(2 * n + 1) {}
	inline int rev(int x) {
		return x > n ? x - n : x + n;
	}
	void addEdge(int u, int v) {
		g[u].push_back(v);
	}
	void addIf(int u, int v) {
		addEdge(u, v);
		addEdge(rev(v), rev(u));
	}
	void addOr(int u, int v) {
		addIf(rev(u), v);
	}
	void setTrue(int u) {
		addEdge(rev(u), u);
	}
	void setFalse(int u) {
		addEdge(u, rev(u));
	}
	vector<bool> query() {
		SCC scc(2 * n, g);
		vector<bool> ret(n + 1);
		for (int i = 1; i <= n; i++) {
			if (scc.same(i, rev(i))) return {};
			ret[i] = scc.find(i) < scc.find(rev(i));
		}
		return ret;
	}
};