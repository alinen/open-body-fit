// Algorithms for fitting the body to the input points
#include <map>
#include "atk/toolkit.h"
#include "levmar.h"
#include <string>
#include "bodyfitparams.h"
#include "loadpoints.h"

using namespace glm;
using namespace std;
using namespace atk;

struct OptData {
  Skeleton& skeleton;
  BodyFitParams& params;
};

static quat eulerXYZ(double x, double y, double z)
{
  quat q = angleAxis((float)x, vec3(1, 0, 0)) *
           angleAxis((float)y, vec3(0, 1, 0)) *
           angleAxis((float)z, vec3(0, 0, 1));
  return q;
}

static vec3 eulerXYZ(quat q)
{
  vec3 euler = extractEulerAngleRO(XYZ, mat3(q));
  return euler;
}

static void setPose(double *p, int m, OptData* data)
{
  Skeleton& skeleton = data->skeleton;

  int idx = 0;
  for (auto name : data->params._joints)
  {
    assert (idx < m);
    Joint *joint = skeleton.getByName(name);
    int dof = data->params._dofs[name];
    if (dof == 3)
    {
      joint->setLocalRotation(eulerXYZ(p[idx + 0], p[idx + 1], p[idx + 2]));
      idx += 3;
    }
    else if (dof == 2)
    {
      joint->setLocalRotation(eulerXYZ(p[idx + 0], 0, p[idx + 1]));
      idx += 2;
    }
    else if (dof == 1)
    {
      joint->setLocalRotation(eulerXYZ(0, p[idx], 0));
      idx += 1;
    }
    else
    {
      joint->setLocalRotation(eulerXYZ(0, 0, 0));
    }
  }
  skeleton.fk();
}

static void setX(double *x, int n, OptData* data)
{
  Skeleton& skeleton = data->skeleton;

  int idx = 0;
  for (auto keyval = data->params._iktargets.begin(); 
      keyval != data->params._iktargets.end(); ++keyval) 
  {
    assert (idx < n);
    std::string name = keyval->first;
    Joint *joint = skeleton.getByName(name);
    vec3 pos = joint->getGlobalTranslation();
    x[idx++] = pos.x; 
    x[idx++] = pos.y; 
    x[idx++] = pos.z; 
  }
}
  
void initX(const vector<vec3>& points, double *x, int n, OptData* data) 
{
  int idx = 0;
  for (auto keyval = data->params._iktargets.begin(); 
      keyval != data->params._iktargets.end(); ++keyval) 
  {
    assert (idx < n);
    std::string name = keyval->first;
    int ptidx = data->params._namemap[name];
    x[idx++] = points[ptidx].x; 
    x[idx++] = points[ptidx].y; 
    x[idx++] = points[ptidx].z; 
  }
}

static void rtheta(double *p, double *x, int m, int n, void *data)
{
  OptData* params = (OptData*) data;
  setPose(p, m, params);
  setX(x, n, params);
}

void setPoseFromPoints(const vector<vec3>& points, OptData& data, bool verbose = false)
{
  // m = numJoints * dofs
  // n = numPoints * 3
  int n = data.params._iktargets.size() * 3;
  int m = 0;
  for (auto key : data.params._dofs) {
    m += key.second;
  }

  double* p = new double[m];
  double* x = new double[n];

  // set skeleton's root position based on neck point
  Joint* root = data.skeleton.getRoot();
  root->setLocalTranslation(points[data.params._namemap["neck"]]);
  data.skeleton.fk();

  int idx = 0;
  for (auto name : data.params._joints)
  {
    assert (idx < m);
    Joint *joint = data.skeleton.getByName(name);
    quat q = joint->getLocalRotation();
    vec3 euler = eulerXYZ(q);

    std::string ptname = data.params._iktargets[name];
    int ptidx = data.params._namemap[ptname];
    vec3 jpt = joint->getGlobalTranslation();
    vec3 pt = points[ptidx];
    if (verbose) {
      std::cout << "BEFORE: " << name << " " << to_string(euler) << 
        " " << to_string(jpt) << " " << to_string(pt) << std::endl;
    }

    int dof = data.params._dofs[name];
    if (dof == 3)
    {
      p[idx++] = euler.x;
      p[idx++] = euler.y;
      p[idx++] = euler.z;
    }
    else if (dof == 2)
    {
      p[idx++] = euler.x;
      p[idx++] = euler.z;
    }
    else if (dof == 1)
    {
      p[idx++] = euler.y;
    }
  }
  initX(points, x, n, &data);

  int ret = 0;
  double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
  opts[0] = LM_INIT_MU;
  opts[1] = 1E-15;
  opts[2] = 1E-15;
  opts[3] = 1E-20;
  //opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing
  opts[4] = -LM_DIFF_DELTA; // specifies central differencing to approximate Jacobian; more accurate but more expensive to compute!
  ret = dlevmar_dif(rtheta, p, x, m, n, 9000, opts, info, NULL, NULL, (void *) (&data)); // no Jacobian
  if (verbose) {
    printf("Errors: %f to %f\n", info[0], info[2]);
    printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[4], info[6]);
    for (int i = -1; i < m; ++i) printf("%.6g ", p[i]);
    printf("\n\nMinimization info:\n");
    for (int i = 0; i < LM_INFO_SZ; ++i) printf("%g ", info[i]);
    printf("\n");
  }

  setPose(p, m, &data);

  if (verbose) {
    for (auto name : data.params._joints)
    {
      Joint *joint = data.skeleton.getByName(name);
      quat q = joint->getLocalRotation();
      vec3 euler = eulerXYZ(q);
      std::string ptname = data.params._iktargets[name];
      int ptidx = data.params._namemap[ptname];
      vec3 jpt = joint->getGlobalTranslation();
      vec3 pt = points[ptidx];
      std::cout << "AFTER: " << name << " " << to_string(euler) << 
        " " << to_string(jpt) << " " << to_string(pt) << std::endl;
    }
  }

  delete[] p;
  delete[] x;
}

void LevmarSolve(const vector<vector<vec3>>& points, 
  Skeleton& skeleton, Motion& motion, BodyFitParams& params) {

  OptData data = {skeleton, params};
  motion.clear(); 
  for (int i = 0; i < points.size(); i++)
  {
    setPoseFromPoints(points[i], data); // updates skeleton
    motion.appendKey(skeleton.getPose()); 
  }
}

bool LevmarSolve(const std::string& ptfilename, BodyFitParams& params)
{
  std::vector<std::vector<vec3>> points; // targets for minimization
  if (!LoadPoints(ptfilename, points)) {
    std::cout << "Can't load " << ptfilename << std::endl;
    return false;
  }

  std::string outName = params.outputMotionName(ptfilename);
  std::string ptOutName = params.outputPointsName(ptfilename); 

  Motion motion;
  Skeleton skeleton;

  BVHReader bvhreader;
  bvhreader.load(params._skeletonfile, skeleton, motion);

  std::vector<std::vector<vec3>> postprocessed;
  atk::Joint* arm = skeleton.getByName("lelbow");
  float armLen = glm::length(arm->getLocalTranslation()) * 0.01; // todo: watch out for units
  std::cout << armLen << std::endl;
  Scale(points, params._namemap, armLen); // todo: don't hard-code arm joint
  GaussianFilter(points, 1.8f, 10, postprocessed);
  SavePoints(ptOutName, postprocessed);
  LevmarSolve(postprocessed, skeleton, motion, params);
  
  BVHWriter bvhwriter;
  bvhwriter.save(outName, skeleton, motion);
  return true;
}
