//Trie
const int MAX=1000100;

int trie[MAX][26];
int leaf[MAX];

int sz=1;
void add_string(string s){
    int v=0;
    for(auto c:s){
        if(!trie[v][c-'a']){
            trie[v][c-'a']=sz;
            sz++;
        }
        v=trie[v][c-'a'];
    }
    leaf[v]=1;
}