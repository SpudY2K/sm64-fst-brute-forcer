#ifndef STICK_H_CL
#define STICK_H_CL

float find_closest_mag(float target) {
    int minIdx = -1;
    int maxIdx = magCount;

    while (maxIdx > minIdx + 1) {
        int midIdx = (maxIdx + minIdx) / 2;

        if (target < magSet[midIdx]) {
            maxIdx = midIdx;
        }
        else {
            minIdx = midIdx;
        }
    }

    if (minIdx == -1) {
        return magSet[maxIdx];
    }
    else if (maxIdx == magCount) {
        return magSet[minIdx];
    }
    else if (target - magSet[minIdx] < magSet[maxIdx] - target) {
        return magSet[minIdx];
    }
    else {
        return magSet[maxIdx];
    }
}

#endif