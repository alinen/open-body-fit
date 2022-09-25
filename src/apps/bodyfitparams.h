#ifndef BODY_FIT_PARAMS_H_
#define BODY_FIT_PARAMS_H_

#include <vector>
#include <string>
#include <utility>
#include <map>

class BodyFitParams {
 public:
  BodyFitParams();
  bool load(const std::string& inifile);

  static std::string outputMotionName(
    const std::string& ptfilename);

  static std::string outputPointsName(
    const std::string& ptfilename);
  
  static std::string outputPrefix(
    const std::string& ptfilename);

  float _height;
  float _weight;
  std::string _skeletonfile;
  std::map<std::string, std::string> _iktargets; // map from joint names to point names
  std::map<std::string, int> _dofs; // map for joint names to number of dofs
  std::map<std::string, int> _namemap; // map from point names to point indices 
  std::vector<std::string> _joints; // joints to solve rotations for
  
  typedef std::tuple<std::string, std::string> Line;
  std::vector<Line> _connections;
};

#endif