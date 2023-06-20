int LCS(string &s,string &t){
    int n=s.size();
    int m=t.size();
    vector<vector<int> >dp(n+1,vector<int>(m+1));
    for(int i=0;i<n+1;i++){
        for(int j=0;j<m+1;j++){
            if(i<n)dp[i+1][j]=max(dp[i][j],dp[i+1][j]);
            if(j<m)dp[i][j+1]=max(dp[i][j],dp[i][j+1]);
            if(i<n&&j<m){
                if(s[i]==t[j]){
                    dp[i+1][j+1]=max(dp[i][j]+1,dp[i+1][j+1]);
                }else{
                    dp[i+1][j+1]=max(dp[i][j],dp[i+1][j+1]);
                }
            }
        }
    }
    return dp[n][m];
}
 