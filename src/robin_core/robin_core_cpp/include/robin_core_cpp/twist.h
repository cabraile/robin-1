#ifndef ROBIN_CORE_TWIST_H
#define ROBIN_CORE_TWIST_H

#include <robin_core_cpp/frames.h>
#include <robin_core_cpp/point.h>
namespace robin_core
{
struct Twist2d
{
    Frame   frame{Frame::UNDEFINED};
    Point2d linear{};
    double  angular{};
};

} // namespace robin_core

#endif // ROBIN_CORE_TWIST_H