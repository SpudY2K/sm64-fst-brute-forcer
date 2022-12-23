#ifndef TRIG_H_CL
#define TRIG_H_CL

extern constant float gSineTableG[4096];
extern constant float gCosineTableG[4096];
extern constant short gArctanTableG[8192];

inline short atan2_lookupG(float z, float x) {
    return (x == 0)? 0x0000 : gArctanTableG[ushort(float(float(z / x) * 1024.0 + 0.5))];
}

inline short atan2sG(float z, float x) {
    short angle = 0;

    if (x >= 0) {
        if (z >= 0) {
            if (z >= x) {
                angle = atan2_lookupG(x, z);
            }
            else {
                angle = 0x4000 - atan2_lookupG(z, x);
            }
        }
        else {
            z = -z;

            if (z < x) {
                angle = 0x4000 + atan2_lookupG(z, x);
            }
            else {
                angle = 0x8000 - atan2_lookupG(x, z);
            }
        }
    }
    else {
        x = -x;

        if (z < 0) {
            z = -z;

            if (z >= x) {
                angle = 0x8000 + atan2_lookupG(x, z);
            }
            else {
                angle = 0xC000 - atan2_lookupG(z, x);
            }
        }
        else {
            if (z < x) {
                angle = 0xC000 + atan2_lookupG(z, x);
            }
            else {
                angle = -atan2_lookupG(x, z);
            }
        }
    }

    return angle;
}

#endif