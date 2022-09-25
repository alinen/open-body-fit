#ifndef ATK_BVHWRITER_H_
#define ATK_BVHWRITER_H_

#include <map>
#include <string>
#include <fstream>
#include "atk/skeleton.h"
#include "atk/motion.h"

namespace atk {
class BVHWriter {
public:
  BVHWriter();
  virtual ~BVHWriter();

  bool save(const std::string& filename,
      Skeleton& skeleton, Motion& motion) const;

protected:

  glm::quat computeBVHRot(float r1, float r2, float r3, atk::RotOrder roo) const;
  bool saveSkeleton(std::ofstream &outFile, Skeleton& skeleton) const;
  bool saveJoint(std::ofstream &outFile, Skeleton& skeleton, 
      Joint *pParent, std::string prefix) const;

  bool saveMotion(std::ofstream &outFile, 
      Skeleton& skeleton, Motion& motion) const;

  void saveFrame(std::ofstream& outFile, 
      Skeleton& skeleton, Pose& pose) const;
};
}
#endif
