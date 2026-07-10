#pragma once

#include <iomanip>
#include <ostream>

namespace vins
{

inline void writeTumPose(std::ostream &stream,
                         double timestamp,
                         double tx,
                         double ty,
                         double tz,
                         double qx,
                         double qy,
                         double qz,
                         double qw)
{
    stream << std::fixed << std::setprecision(9)
           << timestamp << " "
           << tx << " "
           << ty << " "
           << tz << " "
           << qx << " "
           << qy << " "
           << qz << " "
           << qw << '\n';
}

} // namespace vins
