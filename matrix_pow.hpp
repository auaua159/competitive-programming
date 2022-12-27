template< typename T >
vector<vector< T > > mul(vector<vector< T > > &A, vector< vector< T > > &B) {
    vector<vector< T > > C(A.size(), vector< T >(B[0].size()));
    for(int64 i = 0; i < A.size(); i++){
        for(int64 j = 0; j < B[0].size(); j++){
            for(int64 k = 0; k < B.size(); k++) {
                C[i][j] = (C[i][j] + (A[i][k] * B[k][j]));
            }
        }
    }
    return C;
}
template< typename T >
vector<vector< T > > pow(vector<vector< T > > A, T k) {
    vector<vector< T > > B(A.size(), vector< T >(A.size()));
    for(int64 i = 0; i < A.size(); i++)B[i][i]=1;
    while(k>0){
        if(k&1)B = mul(B, A);
        A = mul(A,A);
        k >>= 1;
    }
    return B;
}