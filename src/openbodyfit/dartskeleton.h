#include "physicsskeleton.h"

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>

class DartSkeleton : public PhysicsSkeleton
{
 public:
  DartSkeleton();
  virtual ~DartSkeleton();

  virtual void init(const std::string& bvhfile, float height = 1.6002, 
    float weight = 67.9, MoUnit unit = CM) override;

  void update(int frame); // todo: update based on elapsed time
  void computeInverseDynamics(int frame);
  void save(const std::string& prefix);

 protected:
  void loadModel();

  void createBox(const dart::dynamics::BodyNodePtr& bn, atk::Joint* joint, 
    const glm::vec3& offset, float radius);
  bool isHandJoint(atk::Joint* joint);
  void setGeometry(const dart::dynamics::BodyNodePtr& bn, atk::Joint* joint, double ballR);
  dart::dynamics::BodyNode* makeRootBody(const dart::dynamics::SkeletonPtr& pendulum, atk::Joint* joint);
  dart::dynamics::BodyNode* addBody(const dart::dynamics::SkeletonPtr& pendulum, 
    dart::dynamics::BodyNode* parent, atk::Joint* joint);

 public:
  dart::dynamics::SkeletonPtr biped;
};
