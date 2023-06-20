#include <bits/stdc++.h>
using namespace std;

//#define int long long
#define REP(i,m,n) for(int i=(m);i<(n);i++)
#define rep(i,n) REP(i,0,n)
#define pb push_back
#define all(a) a.begin(),a.end()
#define rall(c) (c).rbegin(),(c).rend()
#define mp make_pair
#define endl '\n'
//#define vec vector<ll>
//#define mat vector<vector<ll> >
#define fi first
#define se second
//#define double long double
typedef long long ll;
typedef unsigned long long ull;
typedef pair<ll,ll> pll;
//typedef long double ld;
typedef complex<double> Complex;
const ll INF=1e9+7;
const ll MOD=998244353;
const ll inf=INF*INF;
const ll mod=MOD;
const ll inv2=499122177;
const ll MAX=200010;
const double PI=acos(-1.0);
typedef vector<vector<ll> > mat;
typedef vector<ll> vec;



void solve(){
    ll n,k;cin>>n>>k;
    string s;cin>>s;
    vector<pair<char,ll> >ls(0);
    ll cnt=0;
    char la=s[0];
    rep(i,n){
        if(s[i]==la)cnt++;
        else{
            ls.pb(mp(la,cnt));
            cnt=1;
        }
        la=s[i];
    }

    ls.pb(mp(la,cnt));
    ll cntx=0;
    rep(i,n){
        if(s[i]=='X')cntx++;
    }
    if(cntx<=k){
        ll res=k-cntx;
        if(res==0){
            cout<<n-1<<endl;
            return;
        }
        vector<ll>ys(0);
        rep(i,n){
            if(s[i]=='Y')ys.pb(i);
        }
        ll ans=n-ys[ys.size()-res];
        rep(i,ys.size()){
            if(res-1==i){
                ans=min(ans,ys[i]+1);
                break;
            }
            ll tmp=ys[i]+1;
            tmp+=(n-ys[ys.size()-(res-i-1)]);
            ans=min(ans,tmp);
        }
        ans=n-ans;
        rep(i,ls.size()){
            if(ls[i].fi=='X')ans=max(ans,ls[i].se);
        }
        cout<<max(ans-1,0LL)<<endl;
        return;
    }
    ll nowx=0,r=0,now=0,ans=0;
    rep(l,ls.size()){
        while(r<ls.size()&&nowx<=k){
            if(ls[r].fi=='X'){
                nowx+=ls[r].se;
            }
            now+=ls[r].se;
            r++;
        }
        if(nowx>k){
            ans=max(ans,now+k-nowx);
        }else{
            ans=max(ans,now);
        }
        now-=ls[l].se;
        if(ls[l].fi=='X')nowx-=ls[l].se;
    }
    cout<<max(ans-1,0LL)<<endl;
}

signed main(){
    cin.tie(0);
    ios::sync_with_stdio(false);
    solve();
}