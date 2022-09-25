#include "bodyfitparams.h"
#include "INIReader.h"
#include <set>
#include <iostream>
#include <string>
#include <bits/stdc++.h>

using namespace std;

BodyFitParams::BodyFitParams() {}

bool BodyFitParams::load(const std::string& inifilename) {
  INIReader reader(inifilename);

  _height = reader.GetFloat("skeleton", "height", 1.6002); // 1.6002 ~ 5ft 3 inches
  _weight = reader.GetFloat("skeleton", "weight", 67.9); // kilograms
  _skeletonfile = reader.Get("skeleton", "bvhname", "");

  set<string> dofs = reader.Keys("skeleton.dofs");
  for (auto dof : dofs) {
    _dofs[dof] = reader.GetInteger("skeleton.dofs", dof, 3);
    _joints.push_back(dof);
  }

  set<string> iktargets = reader.Keys("skeleton.ik_targets");
  for (auto target : iktargets) {
    _iktargets[target] = reader.Get("skeleton.ik_targets", target, "unknown");
  }

  set<string> ptnames = reader.Keys("points.name_map");
  for (auto name : ptnames) {
    _namemap[name] = reader.GetInteger("points.name_map", name, -1);
  }
  set<string> lines = reader.Keys("points.connections");
  for (auto line : lines) {
    std::string connectionstr = reader.Get("points.connections", line, "");
    std::string endpoint1, endpoint2;

    std::stringstream parser(connectionstr);
    std::getline(parser, endpoint1, ' ');
    std::getline(parser, endpoint2, ' ');
    _connections.push_back(Line{endpoint1, endpoint2});
  }
  

  return true;
}

std::string BodyFitParams::outputMotionName(
  const std::string& ptfilename) {

  std::string outName = outputPrefix(ptfilename);
  outName += ".bvh";
  return outName;
}


std::string BodyFitParams::outputPointsName(
  const std::string& ptfilename) {

  std::string ptOutName = outputPrefix(ptfilename);
  ptOutName += "-postprocess.csv";
  return ptOutName;
}

std::string BodyFitParams::outputPrefix(
  const std::string& ptfilename) {

  int len = ptfilename.size();
  return ptfilename.substr(0, len-4);
}
