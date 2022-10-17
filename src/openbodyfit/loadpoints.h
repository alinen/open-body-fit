#ifndef LOAD_POINTS_H_
#define LOAD_POINTS_H_

extern bool LoadPoints(const std::string &filename,
  std::vector<std::vector<glm::vec3>> &points);

extern bool LoadPoints2D(const std::string &filename,
  std::vector<std::vector<glm::vec2>> &points);

extern void Scale(std::vector<std::vector<glm::vec3>>& points, 
  std::map<std::string, int>& namemap, float armMeters);

extern void GaussianFilter(std::vector<std::vector<glm::vec3>>& points, 
   float sigma, int size, std::vector<std::vector<glm::vec3>>& result);

extern void SavePoints(const std::string& filename, 
  const std::vector<std::vector<glm::vec3>>& points);

#endif