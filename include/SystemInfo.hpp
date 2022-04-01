#ifndef ROBOSIGNAL_SYSTEMINFO_HPP
#define ROBOSIGNAL_SYSTEMINFO_HPP

#include "robosignal_global.hpp"

namespace rc {

class ROBOSIGNALSHARED_EXPORT SystemInfo {
public:
    static double GetCpuTemperature();

public:
    static constexpr const double ImpossibleTemperature = -1000.0;
};

}
#endif
