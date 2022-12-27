template< typename T >
vector< T > bellman_ford( vector<vector<pair<int32, T> > >&G, int32 V, int32 s, T INF) {
    vector< T > dist(V,INF);
    dist[s] = 0;
    for(int32 i = 0; i < V - 1; i++) {
        for(int32 j = 0; j < V; j++) {
            if(dist[j] == INF) continue;
            for(auto &e : G[j]) {
                dist[e.first] = min(dist[e.first], dist[j] + e.second);
            }
        }
    }
    for(int32 i = 0; i < V; i++) {
        if(dist[i] == INF)continue;
        for(auto e : G[i]) {
            if(dist[i] + e.second < dist[e.first])return vector< T >();
        }
    }
    return dist;
}
