#include "dartskeleton.h"

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using glm::vec3;

static const double default_pelvis_radius = 0.05; // m
static const double default_radius = 0.01;  // m

DartSkeleton::DartSkeleton() {}
DartSkeleton::~DartSkeleton() {}

void DartSkeleton::update(int frame) // ASN TODO: Use simulation time
{
  atk::Pose pose = motion.getKey(frame);
  skeleton.setPose(pose);

  for (size_t i = 0; i < biped->getNumJoints(); i++)
  {
    Joint* joint = biped->getJoint(i);

    atk::Joint* bvhJoint = skeleton.getByName(joint->getName());
    assert(bvhJoint != 0);

    glm::quat rotation = bvhJoint->getLocalRotation();
    glm::vec3 euler = atk::extractEulerAngleRO(atk::XYZ, glm::mat3(rotation));

    if (joint == biped->getRootJoint()) {
      vec3 pos = bvhJoint->getLocalTranslation();
      joint->setPosition(3, pos[0]);
      joint->setPosition(4, pos[1]);
      joint->setPosition(5, pos[2]);

      joint->setPosition(0, euler[0]);
      joint->setPosition(1, euler[1]);
      joint->setPosition(2, euler[2]);

    } else {

      // Set joint "position" as XYZ euler angles (local coordinates)
      joint->setPosition(0, euler[0]);
      joint->setPosition(1, euler[1]);
      joint->setPosition(2, euler[2]);
    }


    // Set velocity and acceleration
    vec3 vel = jointVels[bvhJoint->getName()][frame]; 
    joint->setVelocity(0, vel[0]);
    joint->setVelocity(1, vel[1]);
    joint->setVelocity(2, vel[2]);

    vec3 acc = jointAccs[bvhJoint->getName()][frame]; 
    joint->setAcceleration(0, acc[0]);
    joint->setAcceleration(1, acc[1]);
    joint->setAcceleration(2, acc[2]);

    joint->setForce(0, 0.0);
    joint->setForce(1, 0.0);
    joint->setForce(2, 0.0);
  }
  biped->computeForwardKinematics(true, true);
}

void DartSkeleton::save(const std::string& prefix)
{
  PhysicsSkeleton::save(prefix);

  // Generalized coordinates
  std::ofstream ofile(prefix+"_forces.txt");
  ofile << "RootX,RootY,RootZ";
  //ofile << "RootX,RootY,RootZ,RootRotX,RootRotY,RootRotZ";
  for (size_t i = 1; i < biped->getNumJoints(); i++)
  {
    Joint* joint = biped->getJoint(i);
    ofile << "," << joint->getName() << "X" <<
             "," << joint->getName() << "Y" <<
             "," << joint->getName() << "Z"; 
  }
  ofile << std::endl;

  std::ofstream mfile(prefix+"_energy.txt");
  mfile << "kineticEnergy\n";

  for (int frame = 0; frame < motion.getNumKeys(); frame++)
  { 
    computeInverseDynamics(frame);

    Joint* joint = biped->getJoint(0);
    Eigen::VectorXd forces = joint->getForces();
    ofile << /*forces[0] <<","<<forces[1]<<","<<forces[2]<<","<<*/forces[3]<<","<<forces[4]<<","<<forces[5];
    for (size_t i = 1; i < biped->getNumJoints(); i++)
    {
      Joint* joint = biped->getJoint(i);
      forces = joint->getForces();
      ofile << "," << forces[0] << "," << forces[1] << "," << forces[2]; 
    }
    ofile << "\n";

    double kineticEnergy = biped->computeKineticEnergy();
    mfile << kineticEnergy << "\n";
  }
  mfile.close();
  ofile.close();


  // body file
  std::ofstream comfile(prefix+"_body.txt");

  for (size_t i = 0; i < biped->getNumJoints(); i++)
  {
    Joint* joint = biped->getJoint(i);
    BodyNode* bn = joint->getChildBodyNode(); 

    // ASN TODO: Output body parameters
    //Eigen::Matrix3d inertia = 
    //  bn->getShapeNode()->getShape()->computeInertia(
    //      body.getMass(joint->getName()));

    Eigen::Vector3d coms = bn->getLocalCOM();
    float mass = bn->getMass();

    comfile << joint->getName() << "------------\n";
//    comfile << "inertia: \n" << inertia << std::endl;
    comfile << "com: " << coms[0] << "," << coms[1] << "," << coms[2] << 
        " mass: " << mass << std::endl;

     Eigen::Matrix6d inertia = bn->getSpatialInertia();
     comfile << joint->getName() << " spatial inertia\n" << inertia << std::endl;
  }

  comfile.close();
}

void DartSkeleton::createBox(const BodyNodePtr& bn, atk::Joint* joint, const glm::vec3& offset, float radius)
{
  // Create a BoxShape to be used for both visualization and collision checking
  // Dimension shoudl be driven by joint size
  float length = glm::length(offset);
  //std::cout << joint->getName() << " " << length << " " << radius << std::endl;

  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(radius, radius, length)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  // Position should be centered on the limb
  Eigen::Vector3d offsetDir = Eigen::Vector3d(offset[0], offset[1], offset[2]).normalized(); 
  Eigen::Vector3d x,y,z;
  x = Eigen::Vector3d::UnitY().cross(offsetDir).normalized(); 
  if (x.norm() < 0.00001) // y and offsetDir are aligned
  {
    y = offsetDir.cross(Eigen::Vector3d::UnitX());
    y.normalize();
    x = y.cross(offsetDir);
    x.normalize();
  }
  else
  {
    x.normalize();
    y = offsetDir.cross(x);
    y.normalize();
  }
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = offsetDir;
  //std::cout << x << " " << y << " " << offsetDir << std::endl;
  Eigen::Vector3d center = 0.5 * Eigen::Vector3d(offset[0], offset[1], offset[2]);

  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  box_tf.fromPositionOrientationScale(center, R, Eigen::Vector3d::Ones());
  shapeNode->setRelativeTransform(box_tf);
}

void DartSkeleton::setGeometry(const BodyNodePtr& bn, atk::Joint* joint, double ballR)
{
  // Make a shape for the Joint
  std::shared_ptr<EllipsoidShape> ball(
        new EllipsoidShape(sqrt(2) * Eigen::Vector3d(ballR, ballR, ballR)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the geometry/com of the Body
  glm::vec3 coms(0,0,0);
  float mass = 0.0; 
  for (int i = 0; i < joint->getNumChildren(); i++)
  {
    atk::Joint* child = joint->getChildAt(i);
    glm::vec3 offset = child->getLocalTranslation(); 
    float radius = body.getRadius(child->getName());

    float comOffset = body.getCOMProximal(child->getName());
    glm::vec3 comLocal = comOffset * offset;
    float childMass = body.getMass(child->getName());
    if (childMass > 0) 
    {
       coms += comLocal*childMass; 
       mass += childMass;
       createBox(bn, child, offset, radius);
    }
  }
  
  // Move the center of mass to the average com of the children
  if (mass > 0) 
  {
     coms = coms / (float) mass;
     bn->setLocalCOM(Eigen::Vector3d(coms[0], coms[1], coms[2]));
     bn->setMass(mass);
  }
}

BodyNode* DartSkeleton::makeRootBody(const SkeletonPtr& pendulum, atk::Joint* joint)
{
  std::string name = joint->getName();

  FreeJoint::Properties properties;
  properties.mName = name;
  properties.mInitialPositions = Eigen::Vector6d::Zero();
  properties.mActuatorType = Joint::ACCELERATION;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<FreeJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  setGeometry(bn, joint, default_pelvis_radius);
  return bn;
}

BodyNode* DartSkeleton::addBody(const SkeletonPtr& pendulum, 
  BodyNode* parent, atk::Joint* joint)
{
  glm::vec3 offset = joint->getLocalTranslation(); 

  // Set up the properties for the Joint
  EulerJoint::Properties properties;
  properties.mName = joint->getName();
  properties.mAxisOrder = EulerJoint::AxisOrder::XYZ;
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(offset[0], offset[1], offset[2]); // Joint offset
  properties.mActuatorType = Joint::ACCELERATION;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<EulerJoint>(
        parent, properties, BodyNode::AspectProperties(joint->getName())).second;

  // Make a shape for the Joint
  setGeometry(bn, joint, default_radius);
  return bn;
}

bool DartSkeleton::isHandJoint(atk::Joint* joint)
{
  while (joint)
  {
    if (joint->getName().find("Palm") != std::string::npos)
    {
      return true;
    }
    joint = joint->getParent();
  }
  return false;
}

void DartSkeleton::init(const std::string& filename, float height, 
  float weight, MoUnit unit) 
{
  PhysicsSkeleton::init(filename, height, weight, unit);

  biped = Skeleton::create("biped");
  std::map<atk::Joint*, BodyNode*> bodies;
  for (int i = 0; i < skeleton.getNumJoints(); i++) 
  {
    atk::Joint* joint = skeleton.getByID(i);
    if (isHandJoint(joint)) continue;
    atk::Joint* parent = joint->getParent();
    BodyNode* bn = parent? 
      addBody(biped, bodies[parent], joint) : 
      makeRootBody(biped, joint);
    bodies[joint] = bn;
  }
  biped->enableSelfCollisionCheck();
  biped->disableAdjacentBodyCheck();
}

void DartSkeleton::computeInverseDynamics(int frame) 
{
  update(frame);
  biped->computeInverseDynamics(true);
}
