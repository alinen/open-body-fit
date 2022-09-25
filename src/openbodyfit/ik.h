#ifndef IK_H_
#define IK_H_

#include <vector>
#include "atk/toolkit.h"
class BodyFitParams;

extern bool LevmarSolve(const std::string& ptfilename, 
    BodyFitParams& params);

extern void LevmarSolve(
    const std::vector<std::vector<glm::vec3>>& points, 
    atk::Skeleton& skeleton, atk::Motion& motion, BodyFitParams& params); 

#endif