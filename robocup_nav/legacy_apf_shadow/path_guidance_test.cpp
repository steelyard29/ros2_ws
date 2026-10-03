#include "path_guidance.hpp"
#include <cassert>
#include <limits>
int main(){
  auto v=path_guided_velocity(1,0,-.5,0);
  assert(v[0]>.14&&v[1]==0); // symmetric doorway repulsion cannot cancel forward path
  v=path_guided_velocity(0,1,0,-.5);assert(v[1]>.14&&v[0]==0);
  v=path_guided_velocity(1,0,0,.5);assert(v[1]>0&&v[1]<=.08);
  for(int i=0;i<360;++i){double a=i*3.141592653589793/180;
    v=path_guided_velocity(std::cos(a),std::sin(a),-.4,.5);
    assert(std::hypot(v[0],v[1])<=.150000000000001);
    assert(v[0]*std::cos(a)+v[1]*std::sin(a)>0);
  }
  v=path_guided_velocity(0,0,1,1);assert(v[0]==0&&v[1]==0);
  v=path_guided_velocity(1,0,std::numeric_limits<double>::quiet_NaN(),0);
  assert(v[0]==0&&v[1]==0);
}
