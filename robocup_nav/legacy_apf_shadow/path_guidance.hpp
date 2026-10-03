#pragma once
#include <array>
#include <algorithm>
#include <cmath>

// OFFLINE candidate only. The caller MUST gate fresh/valid lidar and run
// independent map-backed footprint+stopping checks after this function.
// A* gives forward progress; retain the legacy vector's lateral component.
// 0.08m/s lateral and 0.15m/s norm are commissioning limits, not measured
// vehicle dynamics. This helper does not authorize passage or physical motion.
inline std::array<double,2> path_guided_velocity(double dx,double dy,double ax,double ay) {
  if(!std::isfinite(dx)||!std::isfinite(dy)||!std::isfinite(ax)||!std::isfinite(ay))return {0.,0.};
  const double d=std::hypot(dx,dy);
  if(d<1e-6)return {0.,0.};
  const double tx=dx/d,ty=dy/d;
  const double sideways=std::clamp(-ty*ax+tx*ay,-.08,.08);
  const double forward=std::min(.15,.7*d);
  double x=forward*tx-sideways*ty,y=forward*ty+sideways*tx;
  const double n=std::hypot(x,y),scale=n>.15?std::nextafter(.15,0.)/n:1.;
  return {x*scale,y*scale};
}
