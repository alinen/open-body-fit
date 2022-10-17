#include <iostream>
#include <sstream>
#include <fstream>
#include <regex>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include "agl/aglm.h"

using glm::vec3;
using glm::vec2;
bool LoadPoints(const std::string& filename,
   std::vector<std::vector<glm::vec3>>& points)
{
   std::ifstream file(filename, std::ios::in);
   if (!file.is_open())
   {
      std::cout << "Cannot open file: " << filename << std::endl;
      return false;
   }

   points.clear();
   std::string line;
   while (getline(file, line))
   {
      std::stringstream tokenizer(line);
      std::regex whitespace("\\s+");
      std::string token;

      std::vector<float> lineData;
      while (getline(tokenizer, token, ','))
      {
         token = std::regex_replace(token, whitespace, "");
         float value = atof(token.c_str());
         lineData.push_back(value);
      }

      int numValues = lineData.size();
      assert (numValues % 3 == 0);

      std::vector<glm::vec3> linepts;
      for (int i = 0; i < numValues; i += 3) 
      {
         glm::vec3 p(lineData[i+0], lineData[i+1], lineData[i+2]); 
         linepts.push_back(p);
      }
      points.push_back(linepts);
   }
   return true;
}

bool LoadPoints2D(const std::string& filename,
   std::vector<std::vector<glm::vec2>>& points)
{
   std::ifstream file(filename, std::ios::in);
   if (!file.is_open())
   {
      std::cout << "Cannot open file: " << filename << std::endl;
      return false;
   }

   points.clear();
   std::string line;
   while (getline(file, line))
   {
      std::stringstream tokenizer(line);
      std::regex whitespace("\\s+");
      std::string token;

      std::vector<float> lineData;
      while (getline(tokenizer, token, ','))
      {
         token = std::regex_replace(token, whitespace, "");
         float value = atof(token.c_str());
         lineData.push_back(value);
      }

      int numValues = lineData.size();
      assert (numValues % 2 == 0);

      std::vector<glm::vec2> linepts;
      for (int i = 0; i < numValues; i += 2) 
      {
        glm::vec2 p(lineData[i+0], lineData[i+1]);
        linepts.push_back(p);
      }
      points.push_back(linepts);
   }
   return true;
}


void GaussianFilter(std::vector<std::vector<glm::vec3>>& points, float sigma, int size, 
   std::vector<std::vector<glm::vec3>>& result)
{
   float* weights = new float[size];

   int i;
   float sum = 0;
	float mu = (float) (int)(size/2);
	for (i = 0; i < size; i++)
   {
		weights[i] = 1*exp(-pow((i - mu),2)/(2*pow(sigma,2)))/(sigma*sqrt(2*M_PI));
      sum += weights[i];
   }
	//normalize weights
	for (i = 0; i < size; i++) weights[i]/=sum;
 
   // filter curves
   unsigned int nGap = size/2;
   int numChannels = points[0].size();
   int numFrames = points.size();
   result = points; // initialize size
	for (i = 0; i < numChannels; i++)
	{
      for (int j = 0; j < numFrames; j++)
      {
         vec3 average(0,0,0);
         for (int k = 0; k < size; k++)
         {
            int id = std::max<int>(0, std::min<int>(numFrames - 1, j + k - nGap));
            average += weights[k] * points[id][i];
         }
         result[j][i] = average;
      }
   }
   delete[] weights;
}

// NOTE: Function assumes that points are in normalized image coordinates
void Scale(std::vector<std::vector<glm::vec3>>& points, 
   std::map<std::string, int>& namemap, float foreArm)
{
   std::vector<float> forearmDistanceSqr;
   for (int f = 0; f < points.size(); f++)
   {
      vec3 lwrist = points[f][namemap["lwrist"]];
      vec3 lelbow = points[f][namemap["lelbow"]];

      vec3 rwrist = points[f][namemap["rwrist"]];
      vec3 relbow = points[f][namemap["relbow"]];

      forearmDistanceSqr.push_back(glm::distance2(lwrist, lelbow));
      forearmDistanceSqr.push_back(glm::distance2(rwrist, relbow));
   }
   std::sort(forearmDistanceSqr.begin(), forearmDistanceSqr.end());
   float median = sqrt(forearmDistanceSqr[forearmDistanceSqr.size()/2]);
   std::cout << "median arm distance: " << median << std::endl;

   float ratio = foreArm / median;
   //float ratio = 1.6002 * 0.1250 / median;
   ratio = 100 * ratio; // convert from meters to centimeters
   std::cout << "ratio (image 2 meters): " << ratio << std::endl;
   
   // Scale and recenter
   vec3 rootpos = points[0][namemap["thorax"]];
   for (int f = 0; f < points.size(); f++)
   {
      for (int i = 0; i < points[f].size(); i++)
      {
         vec3 p = points[f][i];
         p = ratio * (p - rootpos);
         float z = p[2] + 100; // scale up height so character is above ground
         float y = p[1];
         // make Y up
         p[2] = -y;
         p[1] = z;
         points[f][i] = p;
      }
   }

}

void SavePoints(const std::string& filename, const std::vector<std::vector<glm::vec3>>& points)
{
   std::ofstream ofile(filename);

   for (int f = 0; f < points.size(); f++)
   {
      ofile << points[f][0].x << "," << points[f][0].y << "," << points[f][0].z;
      for (int i = 1; i < points[f].size(); i++)
      {
         ofile << "," << points[f][i].x << "," << points[f][i].y << "," << points[f][i].z;
      }
      ofile << std::endl;
   }
   ofile.close();
}