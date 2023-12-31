//SegmentTree
template< typename Monoid >
struct SegmentTree
{
  using F = function< Monoid(Monoid, Monoid) >;
 
  int sz;
  vector< Monoid > seg;
 
  const F f;
  const Monoid M1;
  int N;
  SegmentTree(int n, const F f, const Monoid &M1) : f(f), M1(M1)
  {
    N = n;
    sz = 1;
    while(sz < n) sz <<= 1;
    seg.assign(2 * sz, M1);
  }
 
  void set(int k, const Monoid &x)
  {
    seg[k + sz] = x;
  }
 
  void build()
  {
    for(int k = sz - 1; k > 0; k--) {
      seg[k] = f(seg[2 * k + 0], seg[2 * k + 1]);
    }
  }
 
  void update(int k, const Monoid &x)
  {
    k += sz;
    seg[k] = x;
    while(k >>= 1) {
      seg[k] = f(seg[2 * k + 0], seg[2 * k + 1]);
    }
  }
 
  Monoid query(int a, int b)
  {
    Monoid L = M1, R = M1;
    for(a += sz, b += sz; a < b; a >>= 1, b >>= 1) {
      if(a & 1) L = f(L, seg[a++]);
      if(b & 1) R = f(seg[--b], R);
    }
    return f(L, R);
  }
  // a[l] + ... + a[r-1] <= x となる最大のr
  int max_right(int l, Monoid x, F g)
  {
      int r = l + sz;
      Monoid now = M1;
      while(1){
          if(g(f(now, seg[r]), x)) {
              int tmp = r + 1;
              if((tmp & (-tmp)) == tmp)return N;
              if(r & 1) {
                  now = f(now, seg[r]);
                  r++;
              }
              r >>= 1;
          }else{
              while(r - sz < 0) {
                  r <<= 1;
                  if(g(f(now, seg[r]), x)) {
                      now = f(now, seg[r]);
                      r++;
                  }
              }
              return r - sz;
          }
      }
  }
  //a[l] + ... + a[r-1] <= x となる最小のl
  int min_left(int r, Monoid x, F g)
  {
      int l = r - 1 + sz;
      int now = M1;
      while(1){
          if(g(f(now, seg[l]), x)){
              if((l&(-l))==l)return 0;
              if((l&1)==0){
                  now = f(now, seg[l]);
                  l--;
              }
              l >>= 1;
          }else{
              while(l - sz < 0) {
                  l <<= 1;
                  l++;
                  if(g(f(now, seg[l]), x)){
                      now = f(now, seg[l]);
                      l--;
                  }
              }
              return l + 1 - sz;
          }
      }
      return l - sz;
  }

  Monoid operator[](const int &k) const
  {
    return seg[k + sz];
  }
};