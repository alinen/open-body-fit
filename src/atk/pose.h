#ifndef ATK_POSE_H_
#define ATK_POSE_H_

#include "atk/glmmath.h"
#include <vector>
#include <iostream>

namespace atk {
class Pose {
public:
  Pose();
  Pose(const Pose& p);
  Pose& operator = (const Pose& p);
  virtual ~Pose();

  static Pose Lerp(const Pose& p1, const Pose& p2, float u);
  static Pose Squad(const Pose& p0, const Pose& p1, 
      const Pose& p2, const Pose& p3, float u);

  friend std::ostream& operator<<(std::ostream& s, const Pose& v);
  void deepCopy(const Pose& p);

public:
  glm::vec3 rootPos;
  std::vector<glm::quat> jointRots;
};
}
#endif

