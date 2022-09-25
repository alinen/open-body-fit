#ifndef Anthropometrics_H_
#define Anthropometrics_H_

#include <map>
#include <string>
#include "atk/toolkit.h"

// Lengths are in meters, weight in kgs, density is kgs/litre
class Anthropometrics
{
public:
    Anthropometrics();
    virtual ~Anthropometrics();

    // mappings from joints to metrics
    enum Segment {Hand, Forearm, UpperArm, Foot, Shank, Thigh, HeadNeck, Trunk};
    typedef std::pair<Segment, float> BodyData;
    typedef std::map<std::string, BodyData> BodyMap;
    BodyMap CMU_Mapping;
    BodyMap MB_Mapping;
    BodyMap ASL_Mapping;
    BodyMap KIN_Mapping;
    BodyMap NSL_Mapping;

    // scale factor: conversion factor needed for converting to meters
    void init(const atk::Skeleton& skeleton, float factor = 1.0);
    void init(const atk::Skeleton& skeleton, float height, float weight, float factor = 1.0);

    // Average weight and density
    float getWeight(float height) const;
    float getBodyDensity(float height, float weight) const;

    // Segment properties
    float getMass(Segment s) const;
    float getMass(Segment s, float totalMass) const;
    float getCOMProximal(Segment s) const;
    float getCOMDistal(Segment s) const;
    float getDensity(Segment s, float bodyDensity) const;

    // joint properties
    float getRadius(const std::string& name) const;
    float getMass(const std::string& name) const;
    float getDensity(const std::string& name) const;
    float getCOMProximal(const std::string& name) const;

    // skeleton properties
    glm::vec3 getDimensions(const atk::Skeleton& skeleton) const;        
    float estimateHeight(const atk::Skeleton& skeleton, int upidx) const;

    // bounding spheres for self-collision testing
    typedef std::pair<glm::vec3, float> BSphere;
    std::map<std::string, BSphere> bspheres;

public:
    typedef std::pair<float,float> Limits;  // min,max for joint in radians
    class DOF 
    {
    public:
        DOF(Limits l) : limits(l) {}
        Anthropometrics::Limits limits; 
        glm::vec3 axis; // depending on the joint type, this can mean a direction vector, or a rotation vector
    };
    // Depending on the number of DOFs, a joint could be a ball, swing, or hinge joint
    std::map<std::string, std::vector<DOF>> jointDOFs; 

   void setupBoneShapes(const atk::Skeleton& skeleton, float h, float totalMass);
   void computeMass(const atk::Skeleton& skeleton);

protected:

    float _height2weightB; // y-intercept
    float _height2weightM; // slope

    float _HandDensityB;
    float _HandDensityM;
    float _ForearmDensityB;
    float _ForearmDensityM;
    float _UpperArmDensityB;
    float _UpperArmDensityM;
    float _FootDensityB;
    float _FootDensityM;
    float _ShankDensityB;
    float _ShankDensityM;
    float _ThighDensityB;
    float _ThighDensityM;
    float _HeadNeckDensityB;
    float _HeadNeckDensityM;
    float _TrunkDensityB;
    float _TrunkDensityM;

    std::map<Segment, float> _mass;
    std::map<Segment, float> _comProximal;
    std::map<Segment, float> _rogProximal;

    // body shape parameters
    float _height;
    float _skeletonMass; // mass of the skeleton (skeleton might only be part of the full body)
    float _bodyMass; // input weight of the subject
    std::map<std::string,float> _aspx; // radius of bone
    std::map<std::string,float> _jmass; // masses
    std::map<std::string,float> _jmassCombo; // masses as sum of children
    std::map<std::string,float> _jdensity; // masses as sum of children
    // COM offset from start of each bone, e.g. comPos = comOffset * joint->LocalTranslation()
    std::map<std::string,float> _comOffset; 

    std::map<std::string,glm::mat3> _jI; // momments of inertia
    std::map<std::string,glm::mat3> _jIlocal; // momments of inertia

};

#endif
