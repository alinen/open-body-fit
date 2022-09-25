#ifndef ATK_BVHREADER_H_
#define ATK_BVHREADER_H_

#include <map>
#include <string>
#include <fstream>
#include "atk/skeleton.h"
#include "atk/motion.h"

namespace atk {
class BVHReader {
 public:
  BVHReader();
  virtual ~BVHReader();

  bool load(const std::string& filename,
    Skeleton& skeleton, Motion& motion) const;

 protected:

  glm::quat computeBVHRot(float r1, float r2, float r3, atk::RotOrder roo) const;

  bool loadSkeleton(std::ifstream &inFile, Skeleton& skeleton) const;

  bool loadJoint(std::ifstream &inFile, Skeleton& skeleton, 
    Joint *pParent, std::string prefix) const;

  bool loadMotion(std::ifstream &inFile, 
    Skeleton& skeleton, Motion& motion) const;

  void loadFrame(std::ifstream& inFile, 
    Skeleton& skeleton, Motion& motion) const;
};
}
#endif
