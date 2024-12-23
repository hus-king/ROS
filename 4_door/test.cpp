#include <iostream>
#include <cmath>
using namespace std;

struct doorfind {
    int start;
    int end;
    int length;
} line[180];

int doorfind(float height[181]) {
    int minus[180];
    for (int i = 0; i < 180; i++) {
        minus[i] = abs(height[i] - height[i + 1]);   // 第 i 个点和第 i+1 个点的高度差
        line[i].length = 1;
    }
    line[0].start = 0;
    int key = 0;
    for (int i = 0; i < 180; i++) {
        if (minus[i] < 0.1) {
            line[key].length++;
            line[key].end = i + 1;
        } else {
            key++;
            line[key].start = i + 1;
        }
    }
    return key + 1; // 返回找到的线段数量
}

int main() {
    float height[181] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 16.05, 16.1, 16.15, 16.2,
        16.25, 16.3, 16.35, 16.4, 16.45, 16.5, 16.55, 16.6, 16.65, 16.7,
        16.75, 16.8, 16.85, 16.9, 16.95, 17, 17.05, 17.1, 17.15, 17.2,
        17.25, 17.3, 17.35, 17.4, 17.45, 17.5, 17.55, 17.6, 17.65, 17.7,
        17.75, 17.8, 17.85, 17.9, 17.95, 18, 18.05, 18.1, 18.15, 18.2,
        18.25, 18.3, 18.35, 18.4, 18.45, 18.5, 18.55, 18.6, 18.65, 18.7,
        18.75, 18.8, 18.85, 18.9, 18.95, 19, 19.05, 19.1, 19.15, 19.2,
        19.25, 19.3, 19.35, 21, 22, 23, 24, 25, 26, 27,
        28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
        38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
        48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
        58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
        68, 69, 70, 71, 72, 73, 73.05, 73.1, 73.15, 73.2,
        73.25, 73.3, 73.35, 73.4, 73.45, 73.5, 73.55, 73.6, 73.65, 73.7,
        73.75, 73.8, 73.85, 73.9, 73.95, 74, 74.05, 74.1, 74.15, 74.2,
        74.25, 74.3, 74.35, 74.4, 74.45, 74.5, 74.55, 74.6, 74.65, 74.7,
        74.75, 74.8, 74.85, 76, 77, 78, 79, 80, 81, 82, 83
    };

    int num_lines = doorfind(height);

    cout << "Number of lines found: " << num_lines << endl;
    for (int i = 0; i < num_lines; i++) {
        cout << "Line " << i + 1 << ": Start = " << line[i].start << ", End = " << line[i].end << ", Length = " << line[i].length << endl;
    }

    // 选取长度最长的两组并记录 start, end 到 key[4] 中
    int key[4] = {-1, -1, -1, -1};
    int max1 = -1, max2 = -1;
    int max1_index = -1, max2_index = -1;

    for (int i = 0; i < num_lines; i++) {
        if (line[i].length > max1) {
            max2 = max1;
            max2_index = max1_index;
            max1 = line[i].length;
            max1_index = i;
        } else if (line[i].length > max2) {
            max2 = line[i].length;
            max2_index = i;
        }
    }

    if (max1_index != -1) {
        key[0] = line[max1_index].start;
        key[1] = line[max1_index].end;
    }
    if (max2_index != -1) {
        key[2] = line[max2_index].start;
        key[3] = line[max2_index].end;
    }

    cout << "Longest lines:" << endl;
    cout << "Line 1: Start = " << key[0] << ", End = " << key[1] << endl;
    cout << "Line 2: Start = " << key[2] << ", End = " << key[3] << endl;

    return 0;
}