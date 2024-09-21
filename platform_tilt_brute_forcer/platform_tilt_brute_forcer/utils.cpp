#include "utils.hpp"
#include <iomanip>
#include <sstream>
#include <math.h>

int getMinPrecision(float num) {
    int precision;
    float m = frexpf(num, &precision);

    if (fabsf(m) == 0.5) precision--;

    precision = (int)ceil(-log10(2.0) * (precision - 24));
    precision = (precision < 0) ? 0 : precision;

    return precision;
}

std::string float2string(float num, int precision) {
    std::ostringstream oss;
    oss << std::setprecision(precision) << std::noshowpoint << num;
    std::string str = oss.str();
    return str;
}

std::string float2string_max(float num) {
    return(float2string(num, getMinPrecision(num)));
}

std::string float2string_max(float num, int buffer) {
    return(float2string(num, getMinPrecision(num) + buffer));
}
