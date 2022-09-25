#include "physicsskeleton.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <map>
#include <vector>
   
using namespace atk;
using glm::vec3;
using glm::vec4;
using glm::quat;
using glm::mat4;
using glm::mat3;

PhysicsSkeleton::PhysicsSkeleton() {}

void PhysicsSkeleton::init(const std::string& bvhfile, float height, 
  float weight, MoUnit unit)
{
  BVHReader reader;
  reader.load(bvhfile, skeleton, motion);

  convertToMeters(unit);
  motion.update(skeleton, 0); // set skeleton pose to first key
  body.init(skeleton, height, weight, 1.0); // 1.0 => Use units in skeleton 
  
  //LoadContacts(filename+".ann");
  computeDerivativesRoot();
  computeDerivativesQuat();
  computeHandTrajectories();
}

void PhysicsSkeleton::convertToMeters(MoUnit unit)
{
  if (unit == MoUnit::M) return;

  float toMeters = MoUnitTable[unit][M];
  for (int i = 0; i < skeleton.getNumJoints(); i++)
  {
    Joint* joint = skeleton.getByID(i);
    joint->setLocalTranslation(toMeters * joint->getLocalTranslation());
  }

  for (int i = 0; i < motion.getNumKeys(); i++)
  {
    Pose pose = motion.getKey(i);
    pose.rootPos = toMeters * pose.rootPos;
    motion.editKey(i, pose);
  }
}

void PhysicsSkeleton::save(const std::string& prefix)
{
  saveSkeletalQuantities(prefix+"_vels.txt", skeleton.getRoot(), rootVels);
  saveSkeletalQuantities(prefix+"_accs.txt", skeleton.getRoot(), rootAccs);
  saveSkeletalQuantities(prefix+"_avels.txt", jointVels);
  saveSkeletalQuantities(prefix+"_aaccs.txt", jointAccs);
  savePositions(prefix+"_positions.txt");
}

void PhysicsSkeleton::computeHandTrajectories()
{
  trajectoryL.clear();
  trajectoryR.clear();

  atk::Joint* handL = skeleton.getByName("lwrist");
  atk::Joint* handR = skeleton.getByName("rwrist");
  atk::Joint* root = skeleton.getRoot();

  for (int frame = 0; frame < motion.getNumKeys(); frame++)
  {
    atk::Pose pose = motion.getKey(frame);
    skeleton.setPose(pose);

    vec3 handPosL = (handL->getGlobalTranslation() - root->getGlobalTranslation()); 
    vec3 handPosR = (handR->getGlobalTranslation() - root->getGlobalTranslation()); 

    trajectoryL.push_back(handPosL);
    trajectoryR.push_back(handPosR);
  }
}

void PhysicsSkeleton::savePositions(const std::string& filename)
{
   std::ofstream file(filename);
   file << "rootX,rootY,rootZ";
   for (int j = 1; j < skeleton.getNumJoints(); j++)
   {
     std::string name = skeleton.getByID(j)->getName(); 
     file << "," << name <<"X," <<name <<"Y," << name <<"Z";
   }
   file << "\n";

   for (int frame = 0; frame < motion.getNumKeys(); frame++)
   {
      atk::Pose pose = motion.getKey(frame);
      skeleton.setPose(pose);

      vec3 p = skeleton.getByID(0)->getGlobalTranslation();
      file << p[0] << "," << p[1] << "," << p[2];
      for (int j = 1; j < skeleton.getNumJoints(); j++)
      {
        p = skeleton.getByID(j)->getGlobalTranslation();
        file << "," << p[0] << "," << p[1] << "," << p[2];
      }
      file << "\n";
   }

   file.close();
} 

void PhysicsSkeleton::saveSkeletalQuantities(
  const std::string& filename, const JointVecMap& data)
{
    std::ofstream ofile(filename);
    ofile << skeleton.getRoot()->getName() << "X" << "," 
          << skeleton.getRoot()->getName() << "Y" << "," 
          << skeleton.getRoot()->getName() << "Z";
    for (size_t i = 1; i < skeleton.getNumJoints(); i++)
    {
      atk::Joint* joint = skeleton.getByID(i);
      ofile << "," << joint->getName() << "X"  
            << "," << joint->getName() << "Y"  
            << "," << joint->getName() << "Z" ;
    }
    ofile << "\n";
    for (size_t f = 0; f < motion.getNumKeys(); f++)
    {
       atk::Joint* root = skeleton.getByID(0);
       vec3 val = data.at(root->getName())[f];
       ofile << val.x << "," << val.y << "," << val.z;
       for (size_t i = 1; i < skeleton.getNumJoints(); i++)
       {
         atk::Joint* joint = skeleton.getByID(i);
         val = data.at(joint->getName())[f];
         ofile << "," << val.x << "," << val.y << "," << val.z;
       }
       ofile << "\n";
    }
    ofile << std::endl;
    ofile.close();
}

void PhysicsSkeleton::saveSkeletalQuantities(const std::string& filename, 
   const atk::Joint* joint, const std::vector<vec3>& data)
{
    std::ofstream ofile(filename);
    ofile << joint->getName() << "X" << "," 
          << joint->getName() << "Y" << "," 
          << joint->getName() << "Z" << "\n";
    for (size_t f = 0; f < motion.getNumKeys(); f++)
    {
       vec3 val = data[f];
       ofile << val.x << "," << val.y << "," << val.z << "\n";
    }
    ofile.close();
}

void PhysicsSkeleton::computeDerivativesQuat()
{
  // If the motion has a single frame, no work to do
  if (motion.getNumKeys() == 1)
  {
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
      atk::Joint* joint = skeleton.getByID(i);
      jointVels[joint->getName()] = std::vector<vec3>(1, vec3(0,0,0));
      jointAccs[joint->getName()] = std::vector<vec3>(1, vec3(0,0,0));
    }
    return;
  }

  int numFrames = motion.getNumKeys(); // note: need to convert from uint to int
  float dt = motion.getDeltaTime();

  int winsize = 3;
  float scale = 1/(12*dt);
  float weights[5] = {1, -8, 0, 8, -1};

  std::map<std::string, std::vector<vec4>> qvels;
  for (int jointid = 0; jointid < skeleton.getNumJoints(); jointid++)
  {
    atk::Joint* joint = skeleton.getByID(jointid);
    std::vector<vec3> vels;
    for (int frame = 0; frame < numFrames; frame++)
    {
      quat q = motion.getKey(frame).jointRots[joint->getID()];
      
      std::vector<quat> data;
      for (int jj = frame - winsize+1; jj < frame + winsize; jj++)
      {
        int idx = std::max<int>(0, std::min<int>(numFrames-1, jj));
        quat qq = motion.getKey(idx).jointRots[joint->getID()];
        data.push_back(qq);
      }

      vec4 qvel(0,0,0,0); // compute q rate
      for (int jj = 0; jj < data.size(); jj++)
      {
        quat qi = data[jj];
        if (glm::dot(q,qi) < 0) qi = -qi;
        qvel += (weights[jj] * vec4(qi.w, qi.x, qi.y, qi.z));
      }
      qvel = scale * qvel;
      qvels[joint->getName()].push_back(qvel);

      // convert to angular velocity
      mat4 Qqvel(vec4(q.w, -q.x, -q.y, -q.z),
                  vec4(q.x, q.w, -q.z, q.y),
                  vec4(q.y, q.z, q.w, -q.x),
                  vec4(q.z, -q.y, q.x, q.w));

      vec4 tmp = 2.0f * Qqvel * qvel;
      vec3 avel(tmp[1], tmp[2], tmp[3]);
      vels.push_back(avel);
    }
    jointVels[joint->getName()] = vels;
  }

  std::map<std::string, std::vector<vec4>>::iterator it;
  for (it = qvels.begin(); it != qvels.end(); ++it)
  {
      atk::Joint* joint = skeleton.getByName(it->first);
      std::vector<vec3> accs;
      for (int ii = 0; ii < it->second.size(); ii++)
      {
          quat q = motion.getKey(ii).jointRots[joint->getID()];

          std::vector<vec4> data;
          for (int jj = ii-winsize+1; jj < ii+winsize; jj++)
          {
              int idx = std::max<int>(0, std::min<int>(numFrames-1, jj));
              vec4 qqrate = it->second[idx];
              data.push_back(qqrate);
          }

          vec4 qvel(0,0,0,0); // compute q rate
          for (int jj = 0; jj < data.size(); jj++)
          {
              qvel += (weights[jj]*data[jj]);
          }
          qvel = qvel * scale;

          // convert to angular acceleration
          mat4 Qqvel(vec4(q.w, -q.x, -q.y, -q.z),
                      vec4(q.x, q.w, -q.z, q.y),
                      vec4(q.y, q.z, q.w, -q.x),
                      vec4(q.z, -q.y, q.x, q.w));

          vec4 tmp = 2.0f*(Qqvel*qvel);
          vec3 avel(tmp[1], tmp[2], tmp[3]);
          accs.push_back(avel);
      }
      jointAccs[joint->getName()] = accs;
  }
}

void PhysicsSkeleton::computeDerivativesRoot()
{
    // If the motion has a single frame, no work to do
    if (motion.getNumKeys() == 1)
    {
        for (int i = 0; i < skeleton.getNumJoints(); i++)
        {
            rootVels = std::vector<vec3>(1, vec3(0,0,0));
            rootAccs = std::vector<vec3>(1, vec3(0,0,0));
        }
        return;
    }

    int numFrames = motion.getNumKeys();
    rootVels = std::vector<vec3>(numFrames, vec3(0,0,0));
    rootAccs = std::vector<vec3>(numFrames, vec3(0,0,0));

    int winsize = 3;
    float weights[5] = {1, -8, 0, 8, -1};
    float dt = 1.0/motion.getFramerate();

    float scale = 1/(12*dt);
    for (int ii = 0; ii < numFrames; ii++)
    {
        std::vector<vec3> data;
        for (int jj = ii-winsize+1; jj < ii+winsize; jj++)
        {
            int idx = jj;
            if (idx > numFrames-1) idx = idx % numFrames;
            else if (idx < 0) idx = numFrames + idx;
            vec3 pos = motion.getKey(idx).rootPos;
            data.push_back(pos);
        }

        vec3 pvel(0,0,0); // compute q rate
        for (int jj = 0; jj < data.size(); jj++)
        {
            pvel += (weights[jj]*data[jj]);
        }
        pvel = pvel * scale;
        rootVels[ii] = pvel;
    }

    for (int ii = 0; ii < rootVels.size(); ii++)
    {
        std::vector<vec3> data;
        for (int jj = ii-winsize+1; jj < ii+winsize; jj++)
        {
            int idx = jj;
            if (idx > numFrames-1) idx = idx % numFrames;
            else if (idx < 0) idx = numFrames + idx;
            vec3 pos = rootVels[idx];
            data.push_back(pos);
        }

        vec3 pvel(0,0,0); 
        for (int jj = 0; jj < data.size(); jj++)
        {
            pvel += (weights[jj]*data[jj]);
        }
        pvel = pvel * scale;
        rootAccs[ii] = pvel;
    }
}

/*
void ComputeLinearDerivatives(atk::Motion& bvhMotion, atk::Skeleton& bvhSkeleton) 
{
  int winsize = 3;
  float weights[5] = {1, -8, 0, 8, -1};

  float scale = bvhMotion.getFramerate()/12.0;
  int numFrames = bvhMotion.getNumKeys();

  std::map<std::string, std::vector<vec3>> vels;
  std::map<std::string, std::vector<vec3>> accs;
  for (int i = 0; i < bvhSkeleton.getNumJoints(); i++)
  {
    atk::Joint* joint = bvhSkeleton.getByID(i);
    vels[joint->getName()] = std::vector<glm::vec3>(numFrames);
    accs[joint->getName()] = std::vector<glm::vec3>(numFrames);
  }
  for (int frame = 0; frame < bvhMotion.getNumKeys(); frame++)
  {
    for (int i = 0; i < bvhSkeleton.getNumJoints(); i++)
    {
      atk::Joint* joint = bvhSkeleton.getByID(i);
      vec3 vel(0);
      for (int jj = 0; jj < 5; jj++)
      {
        int idx = std::min<int>(bvhMotion.getNumKeys()-1, std::max<int>(0, frame+jj-winsize));
        bvhSkeleton.setPose(bvhMotion.getKey(idx)); // expensive but oh well
        vel += weights[jj] * joint->getGlobalTranslation();
      }
      vel = vel * scale;
      vels[joint->getName()][frame] = vel;
    }
  }

  for (int frame = 0; frame < bvhMotion.getNumKeys(); frame++)
  {
    for (int i = 0; i < bvhSkeleton.getNumJoints(); i++)
    {
      atk::Joint* joint = bvhSkeleton.getByID(i);
      vec3 acc(0);
      for (int jj = 0; jj < 5; jj++)
      {
        int idx = std::min<int>(bvhMotion.getNumKeys()-1, std::max<int>(0, frame+jj-winsize));
        acc += weights[jj] * vels[joint->getName()][idx];
      }
      acc = acc * scale;
      accs[joint->getName()][frame] = acc;
    }
  }
}
*/
