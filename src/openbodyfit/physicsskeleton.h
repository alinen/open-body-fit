#include "atk/toolkit.h"
#include "unitutils.h"
#include "anthropometrics.h"

typedef std::map<std::string, std::vector<glm::vec3>> JointVecMap;

class PhysicsSkeleton
{
 public:
  PhysicsSkeleton();

  virtual void init(const std::string& bvhfile, float height = 1.6002, 
    float weight = 67.9, MoUnit unit = CM);
  virtual void save(const std::string& prefix);

 protected:

  void convertToMeters(MoUnit unit);
  void computeHandTrajectories();
  void computeDerivativesQuat();
  void computeDerivativesRoot();
  void saveSkeletalQuantities(const std::string&, const JointVecMap&);
  void saveSkeletalQuantities(const std::string&, const atk::Joint*, const std::vector<glm::vec3>&);
  void savePositions(const std::string&);

 protected:
  MoUnit _moUnit; // units of input motion

 public:

  atk::Skeleton skeleton;
  atk::Motion motion;
  Anthropometrics body;

  JointVecMap jointVels;
  JointVecMap jointAccs;
  std::vector<glm::vec3> rootVels;
  std::vector<glm::vec3> rootAccs;
  std::vector<glm::vec3> trajectoryL;
  std::vector<glm::vec3> trajectoryR;
};
