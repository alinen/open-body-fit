#include "anthropometrics.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <utility>
#include <algorithm>
#include "atk/joint.h"

using namespace glm;
using namespace atk;

// initialize constants: alternate models can be tried by overriding these values
Anthropometrics::Anthropometrics()
{
    // From http://www.rush.edu/rumc/page-1108048103230.html
    _height2weightB = -61.7542; // y-intercept
    _height2weightM = 73.0766; // slope

    // From Winter Biomechanics Book, Fourth Edition, 2007, Chpt 4
    _mass[Hand] = 0.006;
    _mass[Forearm] = 0.016;
    _mass[UpperArm] = 0.028;
    _mass[Foot] = 0.0145;
    _mass[Shank] = 0.0465;
    _mass[Thigh] = 0.1;
    _mass[HeadNeck] = 0.081;
    _mass[Trunk] = 0.497;

    _comProximal[Hand] = 0.506;
    _comProximal[Forearm] = 0.430;
    _comProximal[UpperArm] = 0.436;
    _comProximal[Foot] = 0.5;
    _comProximal[Shank] = 0.433;
    _comProximal[Thigh] = 0.433;
    _comProximal[HeadNeck] = 0.5; 
    _comProximal[Trunk] = 0.5;

    _rogProximal[Hand] = 0.297; // radius of gyration
    _rogProximal[Forearm] = 0.303;
    _rogProximal[UpperArm] = 0.322;
    _rogProximal[Foot] = 0.475;
    _rogProximal[Shank] = 0.302;
    _rogProximal[Thigh] = 0.323;
    _rogProximal[HeadNeck] = 0.495; 
    _rogProximal[Trunk] = 0.5;

    _HandDensityB = -0.44;
    _HandDensityM = 1.5;
    _ForearmDensityB = -0.0675;
    _ForearmDensityM = 1.125;
    _UpperArmDensityB = 0.4225;
    _UpperArmDensityM = 0.625;
    _FootDensityB = 0.3933;
    _FootDensityM = 0.6667;
    _ShankDensityB = 0.555;
    _ShankDensityM = 0.5;
    _ThighDensityB = 0.3533;
    _ThighDensityM = 0.6667;
    _HeadNeckDensityB = 1.11; //constant for all body densities
    _HeadNeckDensityM = 0;
    _TrunkDensityB = 1.03; // constant
    _TrunkDensityM = 0.0; 

    MB_Mapping["Hips"] = BodyData(Trunk, 0.0);
    MB_Mapping["LeftUpLeg"] = BodyData(Trunk, 0.15);
    MB_Mapping["LeftLeg"] = BodyData(Thigh, 1.0);
    MB_Mapping["LeftFoot"] = BodyData(Shank, 1.0);
    MB_Mapping["LeftToeBase"] = BodyData(Foot, 0.50);
    MB_Mapping["LeftToeBaseSite"] = BodyData(Foot, 0.50);

    MB_Mapping["RightUpLeg"] = BodyData(Trunk, 0.15);
    MB_Mapping["RightLeg"] = BodyData(Thigh, 1.0);
    MB_Mapping["RightFoot"] = BodyData(Shank, 1.0);
    MB_Mapping["RightToeBase"] = BodyData(Foot, 0.50);
    MB_Mapping["RightToeBaseSite"] = BodyData(Foot, 0.50);

    MB_Mapping["Spine"] = BodyData(Trunk, 0.1);
    MB_Mapping["Spine1"] = BodyData(Trunk, 0.2);
    MB_Mapping["Spine2"] = BodyData(Trunk, 0.2);
    MB_Mapping["Spine3"] = BodyData(Trunk, 0.0);
    MB_Mapping["Spine4"] = BodyData(Trunk, 0.0);
    MB_Mapping["Neck"] = BodyData(HeadNeck, 0.1);
    MB_Mapping["Head"] = BodyData(HeadNeck, 0.1);
    MB_Mapping["HeadSite"] = BodyData(HeadNeck, 0.8);

    MB_Mapping["LeftShoulder"] = BodyData(Trunk, 0.05);
    MB_Mapping["LeftArm"] = BodyData(Trunk, 0.05);
    MB_Mapping["LeftForeArm"] = BodyData(UpperArm, 1.0);
    MB_Mapping["LeftHand"] = BodyData(Forearm, 1.0);
    MB_Mapping["LeftHandSite"] = BodyData(Hand, 1.0);

    MB_Mapping["RightShoulder"] = BodyData(Trunk, 0.05);
    MB_Mapping["RightArm"] = BodyData(Trunk, 0.05);
    MB_Mapping["RightForeArm"] = BodyData(UpperArm, 1.0);
    MB_Mapping["RightHand"] = BodyData(Forearm, 1.0);
    MB_Mapping["RightHandSite"] = BodyData(Hand, 1.0);

    CMU_Mapping["root"]=BodyData(Trunk, 0.0);
    CMU_Mapping["lowerback"]=BodyData(Trunk, 0.0); // no length for lowerback
    CMU_Mapping["upperback"]=BodyData(Trunk, 0.3);
    CMU_Mapping["thorax"]=BodyData(Trunk, 0.3);
    CMU_Mapping["lowerneck"]=BodyData(Trunk, 0.2);
    CMU_Mapping["upperneck"]=BodyData(Trunk, 0.05);
    CMU_Mapping["head"]=BodyData(HeadNeck, 0.2);
    CMU_Mapping["headSite"]=BodyData(HeadNeck, 0.75);

    CMU_Mapping["lfemur"]= BodyData(Thigh, 1.0);
    CMU_Mapping["ltibia"]=BodyData(Shank, 1.0);
    CMU_Mapping["lfoot"]=BodyData(Foot, 0.8);
    CMU_Mapping["ltoes"]=BodyData(Foot, 0.1);
    CMU_Mapping["ltoesSite"]=BodyData(Foot, 0.1);

    CMU_Mapping["rfemur"]=BodyData(Thigh, 1.0);
    CMU_Mapping["rtibia"]=BodyData(Shank, 1.0);
    CMU_Mapping["rfoot"]=BodyData(Foot, 0.8);
    CMU_Mapping["rtoes"]=BodyData(Foot, 0.1);
    CMU_Mapping["rtoesSite"]=BodyData(Foot, 0.1);

    CMU_Mapping["lclavicle"]=BodyData(Trunk, 0.1);
    CMU_Mapping["lhumerus"]=BodyData(UpperArm, 1.0);
    CMU_Mapping["lradius"]=BodyData(Forearm, 1.0);
    CMU_Mapping["lwrist"]=BodyData(Hand, 0.6);
    CMU_Mapping["lhand"]=BodyData(Hand, 0.2);
    CMU_Mapping["lthumb"]=BodyData(Hand, 0.05);
    CMU_Mapping["lthumbSite"]=BodyData(Hand, 0.05);
    CMU_Mapping["lfingers"]=BodyData(Hand, 0.05);
    CMU_Mapping["lfingersSite"]=BodyData(Hand, 0.05);

    CMU_Mapping["rclavicle"]=BodyData(Trunk, 0.1);
    CMU_Mapping["rhumerus"]=BodyData(UpperArm, 1.0);
    CMU_Mapping["rradius"]=BodyData(Forearm, 1.0);
    CMU_Mapping["rwrist"]=BodyData(Hand, 0.6);
    CMU_Mapping["rhand"]=BodyData(Hand, 0.2);
    CMU_Mapping["rthumb"]=BodyData(Hand, 0.05);
    CMU_Mapping["rthumbSite"]=BodyData(Hand, 0.05);
    CMU_Mapping["rfingers"]=BodyData(Hand, 0.05);
    CMU_Mapping["rfingersSite"]= BodyData(Hand, 0.05);

    ASL_Mapping["Hips"] = BodyData(Trunk, 0.0);
    ASL_Mapping["LeftUpLeg"] = BodyData(Trunk, 0.2);
    ASL_Mapping["LeftLeg"] = BodyData(Thigh, 1.0);
    ASL_Mapping["LeftFoot"] = BodyData(Shank, 1.0);
    ASL_Mapping["LeftFootHeel"] = BodyData(Foot, 0.1);
    ASL_Mapping["LeftHeelOutside"] = BodyData(Foot, 0.1);
    ASL_Mapping["LeftFootHeelSite"] = BodyData(Foot, 0.4);
    ASL_Mapping["LeftHeelOutsideSite"] = BodyData(Foot, 0.4);

    ASL_Mapping["RightUpLeg"] = BodyData(Trunk, 0.2);
    ASL_Mapping["RightLeg"] = BodyData(Thigh, 1.0);
    ASL_Mapping["RightFoot"] = BodyData(Shank, 1.0);
    ASL_Mapping["RightFootHeel"] = BodyData(Foot, 0.1);
    ASL_Mapping["RightHeelOutside"] = BodyData(Foot, 0.1);
    ASL_Mapping["RightFootHeelSite"] = BodyData(Foot, 0.4);
    ASL_Mapping["RightHeelOutsideSite"] = BodyData(Foot, 0.4);

    ASL_Mapping["Spine"] = BodyData(Trunk, 0.15);
    ASL_Mapping["Spine1"] = BodyData(Trunk, 0.15);
    ASL_Mapping["Neck"] = BodyData(Trunk, 0.1);
    ASL_Mapping["Head"] = BodyData(HeadNeck, 0.2);
    ASL_Mapping["HeadSite"] = BodyData(HeadNeck, 0.8);

    ASL_Mapping["LeftShoulder"] = BodyData(Trunk, 0.05);
    ASL_Mapping["LeftArm"] = BodyData(Trunk, 0.05);
    ASL_Mapping["LeftForeArm"] = BodyData(UpperArm, 1.0);
    ASL_Mapping["LeftHand"] = BodyData(Forearm, 1.0);
    ASL_Mapping["LeftmiddleA"] = BodyData(Hand, 1.0);

    ASL_Mapping["RightShoulder"] = BodyData(Trunk, 0.05);
    ASL_Mapping["RightArm"] = BodyData(Trunk, 0.05);
    ASL_Mapping["RightForeArm"] = BodyData(UpperArm, 1.0);
    ASL_Mapping["RightHand"] = BodyData(Forearm, 1.0);
    ASL_Mapping["RightmiddleA"] = BodyData(Hand, 1.0);

    KIN_Mapping["spineBase"] = BodyData(Trunk,0.0);
    KIN_Mapping["spineMid"] = BodyData(Trunk,0.55);
    KIN_Mapping["spineShoulder"] = BodyData(Trunk,0.25);
    KIN_Mapping["neck"] = BodyData(Trunk,0.1);
    KIN_Mapping["head"] = BodyData(HeadNeck, 1.0);

    KIN_Mapping["shoulderLeft"] = BodyData(Trunk,0.05);
    KIN_Mapping["elbowLeft"] = BodyData(UpperArm, 1.0);
    KIN_Mapping["wristLeft"] = BodyData(Forearm, 1.0);
    KIN_Mapping["palmLeft"] = BodyData(Hand, 0.8);
    KIN_Mapping["indexLeft"] = BodyData(Hand,0.1);
    KIN_Mapping["thumbLeft"] = BodyData(Hand,0.1);

    KIN_Mapping["shoulderRight"] = BodyData(Trunk,0.05);
    KIN_Mapping["elbowRight"] = BodyData(UpperArm, 1.0);
    KIN_Mapping["wristRight"] = BodyData(Forearm, 1.0);
    KIN_Mapping["palmRight"] = BodyData(Hand,0.8);
    KIN_Mapping["indexRight"] = BodyData(Hand,0.1);
    KIN_Mapping["thumbRight"] = BodyData(Hand,0.1);

    NSL_Mapping["torso"] = BodyData(Trunk,1.0);
    NSL_Mapping["neck"] = BodyData(Trunk,0.0);
    NSL_Mapping["head"] = BodyData(HeadNeck, 1.0);

    NSL_Mapping["lshoulder"] = BodyData(Trunk,0.0);
    NSL_Mapping["lelbow"] = BodyData(UpperArm, 1.0);
    NSL_Mapping["lwrist"] = BodyData(Forearm, 1.0);
    NSL_Mapping["lhand"] = BodyData(Hand, 1.0);

    NSL_Mapping["rshoulder"] = BodyData(Trunk,0.0);
    NSL_Mapping["relbow"] = BodyData(UpperArm, 1.0);
    NSL_Mapping["rwrist"] = BodyData(Forearm, 1.0);
    NSL_Mapping["rhand"] = BodyData(Hand, 1.0);

    // TODO: Move to unit test
    // Test mapping: all body parts should sum to 1 (KIN_Mapping is upper body only)
    //std::map<Segment, float> sum = 
    //{ 
    //  {Hand,0.0},    // note: will sum to 2 because 2 hands
    //  {Forearm,0.0}, 
    //  {UpperArm,0.0}, 
    //  {Foot,0.0}, 
    //  {Shank,0.0}, 
    //  {Thigh,0.0}, 
    //  {HeadNeck,0.0}, 
    //  {Trunk,0.0}
    //};
    //for (auto it = KIN_Mapping.begin(); it != KIN_Mapping.end(); it++)
    //{
    //  BodyData data = it->second;
    //  sum[data.first] += data.second;
    //}
    //for (auto it = sum.begin(); it != sum.end(); it++)
    //{
    //  std::cout << it->first << " " << it->second << std::endl;
    //}
}

Anthropometrics::~Anthropometrics()
{
}

void ScaleSkeleton(Skeleton& skeleton, float factor)
{
    // Zero out joints
    skeleton.getRoot()->setLocalTranslation(vec3(0,0,0));
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        Joint* joint = skeleton.getByID(i); 
        joint->setLocalRotation(IdentityQ);
        joint->setLocalTranslation(joint->getLocalTranslation() * (float) factor);
    }
    skeleton.fk(); 
}

void Anthropometrics::init(const Skeleton& inSkeleton, float factor) 
{
    Skeleton skeleton = inSkeleton;
    ScaleSkeleton(skeleton, factor);

    // Find up direction
    int upidx = 2; 
    vec3 dim = getDimensions(skeleton);
    if (dim[1] > dim[2]) upidx = 1;

    float height = estimateHeight(skeleton, upidx);
    float mass = getWeight(height);
    //std::cout << "height: " << height << " weight: " << mass << std::endl; 
    setupBoneShapes(skeleton, height, mass);

    // TODO: Move to unit test
/*
    float d = getBodyDensity(height, mass);
    float volume = mass / d;
    float test1 =  getDensity(Hand, d); 
    float test2 =  getDensity(Forearm, d); 
    float test3 =  getDensity(UpperArm, d); 
    float test4 =  getDensity(Foot, d); 
    float test5 =  getDensity(Shank, d); 
    float test6 =  getDensity(Thigh, d); 
    float test7 =  getDensity(HeadNeck, d); 
    float test8 =  getDensity(Trunk, d); 

    std::vector<float> volumes(8,0);
    volumes[0] =  2*(getMass(Hand, mass) / test1); // note: density * vol = mass
    volumes[1] =  2*(getMass(Forearm, mass) / test2); // note: density * vol = mass
    volumes[2] =  2*(getMass(UpperArm, mass) / test3); // note: density * vol = mass
    volumes[3] =  2*(getMass(Foot, mass) / test4); // note: density * vol = mass
    volumes[4] =  2*(getMass(Shank, mass) / test5); // note: density * vol = mass
    volumes[5] =  2*(getMass(Thigh, mass) / test6); // note: density * vol = mass
    volumes[6] =  getMass(HeadNeck, mass) / test7; // note: density * vol = mass
    volumes[7] =  getMass(Trunk, mass) / test8; // note: density * vol = mass

    float sum = 0;
    for (int i = 0; i < (int) volumes.size(); i++) sum += volumes[i];
    std::cout << sum << " " << volume << std::endl;
*/
}

void Anthropometrics::init(const Skeleton& inSkeleton, 
   float height, float weight, float factor)
{
    Skeleton skeleton = inSkeleton;
    ScaleSkeleton(skeleton, factor);

    // Find up direction
    int upidx = 2; 
    vec3 dim = getDimensions(skeleton);
    if (dim[1] > dim[2]) upidx = 1;

    setupBoneShapes(skeleton, height, weight);
}

void Anthropometrics::setupBoneShapes(const Skeleton& skeleton, float height, float totalMass)
{
    _bodyMass = totalMass;
    Joint* root = skeleton.getRoot();

    float d = getBodyDensity(height, totalMass);
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        Joint* j = skeleton.getByID(i);
        BodyData data = NSL_Mapping[j->getName()];
        _jmass[j->getName()] = getMass(data.first, totalMass) * data.second;
        _jdensity[j->getName()] = getDensity(data.first, d); 
        _comOffset[j->getName()] = getCOMProximal(data.first); 
    }

   computeMass(skeleton);
}

void Anthropometrics::computeMass(const Skeleton& skeleton)
{
   for (int i = 0; i < skeleton.getNumJoints(); i++)
   {
       Joint* j = skeleton.getByID(i);
       float density = _jdensity[j->getName()] * 1000;
       float length = glm::length(j->getLocalTranslation());
       float tmp = density * length * 4; // cuboid
       //float tmp = density * (3.0/4.0) * (1.0/2.0) * M_PI * length; // ellipsoid
       //float tmp = density * M_PI * length; // cylinder
       // density = mass/volume
       // aspx = joint radius: to solve, note that cuboid V = (2*r)*(2*r)*length 0)
       if (tmp > 0 && _jmass[j->getName()] > 0)
       {
           _aspx[j->getName()] = sqrt(_jmass[j->getName()]/tmp); 
       }
       else
       {
           _aspx[j->getName()] = 0.0;
       }
   }

   float fTotalMass = 0;
   for (int i = 0; i < skeleton.getNumJoints(); i++)
   {
        Joint* j = skeleton.getByID(i);
        fTotalMass += _jmass[j->getName()];
    }
    _skeletonMass = fTotalMass;
}

float Anthropometrics::getRadius(const std::string& name) const
{
   return _aspx.find(name)->second;
}

float Anthropometrics::getMass(const std::string& name) const
{
   return _jmass.find(name)->second;
}

float Anthropometrics::getDensity(const std::string& name) const
{
   return _jdensity.find(name)->second;
}

float Anthropometrics::getCOMProximal(const std::string& name) const
{
   return _comOffset.find(name)->second;
}

// cast insensitive search for joint containing any name from names
Joint* findJoint(const Skeleton& skeleton, const std::vector<std::string> names)
{
   for (unsigned int j = 0; j < names.size(); j++)
   {
      std::string name = names[j];
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      //std::cout << name << std::endl;

      for (int i = 0; i < skeleton.getNumJoints(); i++)
      {
         Joint* current = skeleton.getByID(i);
         std::string jointName = current->getName();
         std::transform(jointName.begin(), jointName.end(), jointName.begin(), ::tolower);
         
         //std::cout << "JOINT: " << jointName << std::endl;
         if (jointName.find(name) != std::string::npos)
         {
            return current;
         }
      }
   }


   return 0;
}

// cast insensitive search for end effector of joint containing "name"
Joint* findEndEffector(const Skeleton& skeleton, const std::string& name)
{
   std::string lName = name;
   std::transform(name.begin(), name.end(), lName.begin(), ::tolower);
   for (int i = 0; i < skeleton.getNumJoints(); i++)
   {
      Joint* current = skeleton.getByID(i);
      std::string jointName = current->getName();
      std::transform(jointName.begin(), jointName.end(), jointName.begin(), ::tolower);
      //std::cout << lName << " " << jointName << std::endl;
      if (jointName.find(lName) != std::string::npos)
      {
         // get end effector
         Joint* child = current;
         while (child->getNumChildren() > 0)
         {
            child = child->getChildAt(0);
         }
         return child;
      }
   } 

   return 0;
}

float Anthropometrics::estimateHeight(const Skeleton& skeleton, int upidx) const
{
    Joint* head = findEndEffector(skeleton, "Head");
    if (!head) std::cout << "Cannot find head joint\n";


    Joint* foot = findJoint(skeleton, {"Heel", "Toe", "Foot"});
    if (!foot) std::cout << "Cannot find left foot joint\n";

    float min = 99999999.0;
    float max = -99999999.0;
    if (head && foot)
    {
        min = foot->getGlobalTranslation()[upidx];
        max = head->getGlobalTranslation()[upidx];
    }
    else
    {
        // look for max vertical distance
       for (int i = 0; i < skeleton.getNumJoints(); i++)
       {
            Joint* joint = skeleton.getByID(i);
            vec3 pos = joint->getGlobalTranslation();
            min = std::min<float>(pos[upidx], min);
            max = std::max<float>(pos[upidx], max);
        }
    }

    return max - min;
}

vec3 Anthropometrics::getDimensions(const Skeleton& skeleton) const
{
    vec3 min(9999999999.0,9999999999.0,9999999999.0);
    vec3 max(-9999999999.0,-9999999999.0,-9999999999.0);
    for (int i = 1; i < skeleton.getNumJoints(); i++)
    {
        vec3 pos = skeleton.getByID(i)->getGlobalTranslation();
        min[0] = std::min<float>(min[0], pos[0]);
        min[1] = std::min<float>(min[1], pos[1]);
        min[2] = std::min<float>(min[2], pos[2]);

        max[0] = std::max<float>(max[0], pos[0]);
        max[1] = std::max<float>(max[1], pos[1]);
        max[2] = std::max<float>(max[2], pos[2]);
    }

    vec3 dim(0,0,0);
    dim = max - min;
    return dim; 
}

float Anthropometrics::getWeight(float height) const
{
    return _height2weightB + _height2weightM * height;
}

float Anthropometrics::getBodyDensity(float height, float weight) const
{
    // From Winter Biomechanics Book, Fourth Edition, 2007, Chpt 4
    // via Contini 1972, Body Segment Parameters, Part II
    float c = height / pow(weight, 0.333333333333);
    return 0.69 + 0.9 * c; 
}

float Anthropometrics::getMass(Anthropometrics::Segment s) const
{
    return getMass(s, _bodyMass);
}

float Anthropometrics::getMass(Anthropometrics::Segment s, float totalMass) const
{
    return _mass.find(s)->second * totalMass;
}

float Anthropometrics::getCOMProximal(Anthropometrics::Segment s) const
{
    return _comProximal.find(s)->second;
}

float Anthropometrics::getCOMDistal(Anthropometrics::Segment s) const
{
    return 1.0 - _comProximal.find(s)->second;
}

float Anthropometrics::getDensity(Anthropometrics::Segment s, float bodyDensity) const
{
    switch(s)
    {
    case Hand: return _HandDensityB + _HandDensityM * bodyDensity;
    case Forearm: return _ForearmDensityB + _ForearmDensityM * bodyDensity;
    case UpperArm: return _UpperArmDensityB + _UpperArmDensityM * bodyDensity;
    case Foot: return _FootDensityB + _FootDensityM * bodyDensity;
    case Shank: return _ShankDensityB + _ShankDensityM * bodyDensity;
    case Thigh: return _ThighDensityB + _ThighDensityM * bodyDensity;
    case HeadNeck: return _HeadNeckDensityB + _HeadNeckDensityM * bodyDensity;
    case Trunk: return _TrunkDensityB + _TrunkDensityM * bodyDensity;
    }

    return 1.0; 
}

