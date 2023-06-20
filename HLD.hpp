//Heavy-Light-Decomposition
struct HLD {
    vector< vector< int > > G; 
    vector< int > sz, nxt, id, rev, par;
    const int N;
    int idx;

    HLD(int n):
        N(n), idx(0), G(n), sz(n), nxt(n), id(n), rev(n), par(n){}

    void add_edge(int u, int v){
        G[u].push_back(v);
        G[v].push_back(u);
    }
    
    void dfs_sz(int v,int p){
        par[v] = p;

        sz[v] = 1;
        for(auto &e:G[v]){
            if(e == p)continue;
            dfs_sz(e,v);
            sz[v] += sz[e];
            if(sz[G[v][0]] < sz[e] || G[v][0] == p)swap(G[v][0], e);
        }
        return;
    }

    void dfs_hld(int v, int p){
        id[v] = idx;
        rev[idx] = v;
        idx++;
        for(auto &e:G[v]){
            if(e == p)continue;
            nxt[e] = (G[v][0] == e ? nxt[v] : e);
            dfs_hld(e, v);
        }
    }

    void build(int v){
        dfs_sz(v, -1);
        nxt[v] = v;
        dfs_hld(v, -1);
    }
    //idを返している
    vector< pair<int, int> >query(int u, int v){
        vector< pair<int, int> > res(0);
        while(u != v){
            if(id[u] < id[v])swap(u,v);
            if(id[nxt[u]] <= id[v]){
                res.push_back(make_pair(id[v], id[u]));
                u = v;
                continue;
            }
            res.push_back(make_pair(id[nxt[u]], id[u]));
            u = par[nxt[u]];
        }
        return res;
    }

    int LCA(int u, int v){
        while(u != v){
            if(id[u] < id[v])swap(u,v);
            if(nxt[u] == nxt[v])return v;
            u = par[nxt[u]];
        }
        return u;
    }
};
