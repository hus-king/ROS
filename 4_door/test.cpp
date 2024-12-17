#include <iostream>
#include <cmath>
using namespace std;
int change(int i){
    int out = 90-i;
    if(out<0) out+=360;
    return out;
}
int main(){
    int i = 0;
    for(i = 0;i<180;i++)
        cout<<i<<" "<<change(i)<<endl;
    return 0;
}