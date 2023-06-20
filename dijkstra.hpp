//dijkstra(O(ElogV))
//0-indexed
void dijkstra(ll n,ll s,vector<vector<pll> >&G,vector<ll> &d){
    priority_queue<pll,vector<pll>,greater<pll> >pq;
    rep(i,n){
        d[i]=inf;
    }
    d[s]=0;
    pq.push(mp(0,s));
    while(!pq.empty()){
        pll k=pq.top();pq.pop();
        ll v=k.second;
        if(d[v]<k.first)continue;
        for(auto e:G[v]){
            if(d[e.first]>d[v]+e.second){
                d[e.first]=d[v]+e.second;
                pq.push(mp(d[e.first],e.first));
            }
        }
    }
}
