#include "NavMeshWrapper.h"

#include <vector>
#include <iostream>
#include <map>
#include <cstring>
using namespace std;
using namespace nd;

int main(){
    NavMeshWrapper wraper;
    wraper.init("a.bin");

    float start[3] = {0.94, 0.01, 6.35};
    float end[3] = {262.09, 0.01, 92.39};
    vector<float> path;
    wraper.findPath(start, end, path);


    cout << "[" << start[0] << ", " << start[1] << ", " << start[2] << "] -> ";
    cout << "[" << end[0] << ", " << end[1] << ", " << end[2] << "]" << endl;
    for(unsigned i = 0; i < path.size()/3; i++){
        cout << "[" << path[i * 3 + 0] << ", " << path[i * 3 + 1] << ", " << path[i * 3 + 2] << "]" << endl;
    }
    return 0;
}

