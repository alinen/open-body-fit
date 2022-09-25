#include "atk/bvhreader.h"
#include "atk/glmmath.h"
#include <iostream>

namespace atk {
atk::RotOrder stringToRoo(const std::string& _order) 
{
  std::string order = _order;

  auto empty = std::string::npos;
  if (order.find("Zrotation Xrotation Yrotation") != empty) return ZXY;
  else if (order.find("Zrotation Yrotation Xrotation") != empty) return ZYX;
  else if (order.find("Xrotation Yrotation Zrotation") != empty) return XYZ;
  else if (order.find("Xrotation Zrotation Yrotation") != empty) return XZY;
  else if (order.find("Yrotation Xrotation Zrotation") != empty) return YXZ;
  else if (order.find("Yrotation Zrotation Xrotation") != empty) return YZX;

  std::cerr << "ERROR: Invalid roo " << order << std::endl;
  return XYZ;
}

BVHReader::BVHReader() 
{

}

BVHReader::~BVHReader()
{
}

bool BVHReader::load(const std::string& filename,
    Skeleton& skeleton, Motion& motion) const
{
  std::ifstream inFile(filename.c_str());
  if (!inFile.is_open())
  {
    std::cout << "WARNING: Could not open " << filename.c_str() << std::endl;
    return false;
  }

  skeleton.clear();
  motion.clear();

  bool status = loadSkeleton(inFile, skeleton);
  status = status && loadMotion(inFile, skeleton, motion);
  inFile.close();
  return status;
}

bool BVHReader::loadSkeleton(std::ifstream& inFile, Skeleton& skeleton) const
{
  glm::vec3 offsets;
  std::string readString, jointname;
  int channelCount;

  inFile >> readString;
  if (readString != "HIERARCHY")
    return false;
  inFile >> readString;
  if (readString != "ROOT" && readString != "JOINT")
    return false;
  inFile.get(); //" "
  getline(inFile, jointname);// joint name
  jointname.erase(jointname.find_last_not_of(" \n\r\t")+1); // strip trailing whitespace

  Joint* joint = new Joint(jointname);
  skeleton.addJoint(joint, 0);
  inFile >> readString; // "{"
  inFile >> readString; // "OFFSET"
  inFile >> offsets[0] >> offsets[1] >> offsets[2];
  joint->setLocalTranslation(offsets);
  inFile >> readString;
  if (readString != "CHANNELS")
    return false;
  inFile >> channelCount;
  joint->setNumChannels(channelCount);
  getline(inFile, readString);	// " Xposition Yposition Zposition Zrotation Xrotation Yrotation"
  joint->setRotationOrder(stringToRoo(readString));
  inFile >> readString;
  while (readString != "}")
  {
    if (!loadJoint(inFile, skeleton, joint, readString))
    {
      return false;
    }
    inFile >> readString;
  }
  if (readString != "}") return false;

  skeleton.fk();
  return true;
}

bool BVHReader::loadJoint(std::ifstream &inFile, 
    Skeleton& skeleton, Joint *pParent, std::string prefix) const
{
  std::string readString, jointname;
  glm::vec3 offsets;
  int channelCount;
  if (prefix == "JOINT")
  {
    inFile.get(); //" "
    getline(inFile, jointname);// joint name
    jointname.erase(jointname.find_last_not_of(" \n\r\t")+1); // strip trailing whitespace
    Joint* joint = new Joint(jointname);
    skeleton.addJoint(joint, pParent);
    inFile >> readString; // "{"
    inFile >> readString; // "OFFSET"
    inFile >> offsets[0] >> offsets[1] >> offsets[2];
    joint->setLocalTranslation(offsets);
    inFile >> readString; // "CHANNELS"
    inFile >> channelCount;
    joint->setNumChannels(channelCount);

    getline(inFile, readString);// " Zrotation Xrotation Yrotation"
    joint->setRotationOrder(stringToRoo(readString));

    inFile >> readString; // "Joint" or "}" or "End"
    while (readString != "}")
    {
      if (loadJoint(inFile, skeleton, joint, readString) == false)
        return false;
      inFile >> readString; // "Joint" or "}" or "End"
    }
    return true;
  }
  else if (prefix == "End")
  {
    inFile.get(); //" "
    getline(inFile, jointname);// joint name
    if (jointname.find("Site") != std::string::npos)
    {
      jointname = pParent->getName() + "Site";
    }

    Joint* joint = new Joint(jointname);
    joint->setNumChannels(0);
    skeleton.addJoint(joint, pParent);
    inFile >> readString; // "{"
    inFile >> readString; // "OFFSET"
    inFile >> offsets[0] >> offsets[1] >> offsets[2];
    joint->setLocalTranslation(offsets);
    inFile >> readString; // "}"
    return true;
  }
  else return false;
}

bool BVHReader::loadMotion(std::ifstream& inFile, 
    Skeleton& skeleton, Motion& motion) const
{
  std::string readString;
  int frameCount;
  inFile >> readString;
  if (readString != "MOTION")
    return false;
  inFile >> readString;
  if (readString != "Frames:")
    return false;
  inFile >> frameCount;
  inFile >> readString; // "Frame"
  getline(inFile, readString); // " Time: 0.033333"
  float dt = atof(&(readString.c_str()[6]));

  motion.setFramerate(1.0/dt);

  // Read frames
  for (int i = 0; i < frameCount; i++)
  {
    loadFrame(inFile, skeleton, motion);
  }

  return true;
}

void BVHReader::loadFrame(std::ifstream& inFile, 
    Skeleton& skeleton, Motion& motion) const
{
  float tx, ty, tz, r1, r2, r3;
  Pose pose;
  for (int i = 0; i < skeleton.getNumJoints(); i++)
  {
    tx = ty = tz = 0.0f;
    r1 = r2 = r3 = 0.0f;

    Joint* pJoint = skeleton.getByID(i);
    if (pJoint->getNumChannels() == 6)
    {
      inFile >> tx >> ty >> tz;
      inFile >> r1 >> r2 >> r3;
    }
    else if (pJoint->getNumChannels() == 3)
    {
      inFile >> r1 >> r2 >> r3;
    }
    else
    {
    }

    if (i == 0) // root joint
    {
      pose.rootPos = glm::vec3(tx, ty, tz); 
    }

    glm::quat quat = computeBVHRot(r1, r2, r3, pJoint->getRotationOrder());
    pose.jointRots.push_back(quat);
  }
  motion.appendKey(pose);
}


glm::quat BVHReader::computeBVHRot(float r1, float r2, float r3, 
  atk::RotOrder rotOrder) const
{
  float ry = 0, rx = 0, rz = 0;

  if (rotOrder == atk::XYZ)
  {
    rx = r1; ry = r2; rz = r3;
  }
  else if (rotOrder == atk::XZY)
  {
    rx = r1; rz = r2; ry = r3;
  }
  else if (rotOrder == atk::YXZ)
  {
    ry = r1; rx = r2; rz = r3;
  }
  else if (rotOrder == atk::YZX)
  {
    ry = r1; rz = r2; rx = r3;
  }
  else if (rotOrder == atk::ZXY)
  {
    rz = r1; rx = r2; ry = r3;
  }
  else if (rotOrder == atk::ZYX)
  {
    rz = r1; ry = r2; rx = r3;
  }

  glm::quat q = atk::eulerAngleRO(rotOrder, glm::radians(glm::vec3(rx, ry, rz)));
  return q;
}
}
