#include "atk/bvhwriter.h"
#include <iomanip>

using namespace glm;

namespace atk {
BVHWriter::BVHWriter() {
}

BVHWriter::~BVHWriter() {
}

bool BVHWriter::save(const std::string& filename,
    Skeleton& skeleton, Motion& motion) const {

  std::ofstream outFile(filename.c_str());
  if (!outFile.is_open()) {
    std::cout << "WARNING: Could not open " << 
      filename.c_str() << " for writing" << std::endl;
    return false;
  }

  saveSkeleton(outFile, skeleton);
  saveMotion(outFile, skeleton, motion);

  outFile.close();
  return true;
}


std::string rooToString(atk::RotOrder roo) {
  using namespace atk;
  switch(roo)
  {
    case XYZ: return "Xrotation Yrotation Zrotation";
    case XZY: return "Xrotation Zrotation Yrotation";
    case YXZ: return "Yrotation Xrotation Zrotation";
    case YZX: return "Yrotation Zrotation Xrotation";
    case ZXY: return "Zrotation Xrotation Yrotation";
    case ZYX: return "Zrotation Yrotation Xrotation";
  }
  return "Unknown";
}

bool BVHWriter::saveSkeleton(std::ofstream &outFile, 
    Skeleton& skeleton) const
{
  Joint* pRoot = skeleton.getRoot();
  std::string rooString = rooToString(pRoot->getRotationOrder());

  outFile << "HIERARCHY" << std::endl;
  outFile << "ROOT " << pRoot->getName() << std::endl;
  outFile << "{" << std::endl;
  outFile << "\tOFFSET 0.00 0.00 0.00" << std::endl;
  outFile << "\tCHANNELS 6 Xposition Yposition Zposition " << rooString << std::endl;

  unsigned int childCount = pRoot->getNumChildren();
  for (unsigned int i = 0; i < childCount; i++)
  {
    Joint* pChild = pRoot->getChildAt(i);
    saveJoint(outFile, skeleton, pChild, "\t");
  }
  outFile << "}" << std::endl;
  return true;
}


bool BVHWriter::saveJoint(std::ofstream &outFile, Skeleton& skeleton, 
    Joint *pJoint, std::string prefix) const {
  unsigned int childCount = pJoint->getNumChildren();
  if (childCount > 0) {
    outFile << prefix << "JOINT " << pJoint->getName() << std::endl;
    outFile << prefix << "{" << std::endl;
    vec3 offsets = pJoint->getLocalTranslation();
    outFile << prefix << "\tOFFSET " 
      << offsets[0] << " " << offsets[1] << " " << offsets[2] << std::endl;
    outFile << prefix 
      << "\tCHANNELS 3 " << rooToString(pJoint->getRotationOrder()) << std::endl;
  } else {
    if (pJoint->getName().find("Site") != std::string::npos) {
      outFile << prefix << "End Site" << std::endl;
    } else {
      outFile << prefix << "End " << pJoint->getName() << std::endl;
    }
    outFile << prefix << "{" << std::endl;
    vec3 offsets = pJoint->getLocalTranslation();
    outFile << prefix << "\tOFFSET " 
      << offsets[0] << " " << offsets[1] << " " << offsets[2] << std::endl; 
  }

  for (unsigned int i = 0; i < childCount; i++) {
    Joint* pChild = pJoint->getChildAt(i);
    saveJoint(outFile, skeleton, pChild, prefix + "\t");
  }
  outFile << prefix << "}" << std::endl;
  return true;
}


bool BVHWriter::saveMotion(std::ofstream &outFile, 
    Skeleton& skeleton, Motion& motion) const
{
  outFile << "MOTION" << std::endl;
  outFile << "Frames: " << motion.getNumKeys() << std::endl;
  outFile << "Frame Time: " << motion.getDeltaTime() << std::endl;
  for (int i = 0; i < motion.getNumKeys(); i++)
  {
    Pose pose = motion.getKey(i);
    saveFrame(outFile, skeleton, pose);
  }
  return true;
}


void BVHWriter::saveFrame(std::ofstream& outFile, 
    Skeleton& skeleton, Pose& pose) const
{
  using namespace atk;
  outFile << std::setprecision(6);
  vec3 pos = pose.rootPos;
  outFile << pos[0] << "\t" << pos[1] << "\t" << pos[2];

  for (int i = 0; i < skeleton.getNumJoints(); i++) {
    Joint* joint = skeleton.getByID(i);
    if (joint->getNumChildren() == 0) continue; // skip end sites

    quat q = pose.jointRots[i];
    vec3 a = extractEulerAngleRO(joint->getRotationOrder(), mat3(q));
    a = degrees(a);
    switch (joint->getRotationOrder()) {
      case XYZ: outFile << "\t" << a.x << "\t" << a.y << "\t" << a.z; break;
      case XZY: outFile << "\t" << a.x << "\t" << a.z << "\t" << a.y; break;
      case YXZ: outFile << "\t" << a.y << "\t" << a.x << "\t" << a.z; break;
      case YZX: outFile << "\t" << a.y << "\t" << a.z << "\t" << a.x; break;
      case ZXY: outFile << "\t" << a.z << "\t" << a.x << "\t" << a.y; break;
      case ZYX: outFile << "\t" << a.z << "\t" << a.y << "\t" << a.x; break;
    }
  }
  outFile << std::endl;
}
}
