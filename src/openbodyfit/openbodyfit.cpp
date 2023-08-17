#include "agl/window.h"
#include "agl/image.h"
#include "atk/toolkit.h"
#include "dartskeleton.h"
#include "bodyfitparams.h"
#include "loadpoints.h"
#include "ik.h"
#include <Eigen/Dense>
#include <string>
#include <glm/gtc/type_ptr.hpp>
#include "imgui/imgui.h"                                                         
#include "imgui/implot.h"                                                         
#include "imgui/imgui_impl_glfw.h"                                               
#include "imgui/imgui_impl_opengl3.h" 

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using namespace atk;
using namespace dart;

// Width and height should match image size
unsigned int WIDTH = 720;
unsigned int HEIGHT = 480;

std::string imagedir = "../demo/images/";
mat4 Cmatrix; // todo: compute projection matrix internally
std::vector<std::vector<glm::vec2>> points2D;
std::vector<std::string> annotations;

mat4 eigen2mat4(const Eigen::Isometry3d& x) {
  mat4 out;
  Eigen::MatrixXd in = x.matrix();
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) out[i][j] = in(i,j);
  return out;
}

class VideoViewer : public agl::Window {
 public:
  virtual ~VideoViewer() {
    ImGui_ImplOpenGL3_Shutdown();                                                  
    ImGui_ImplGlfw_Shutdown();                                                     
    ImPlot::DestroyContext();           
    ImGui::DestroyContext();           
  }

  void load(const std::string& ptfilename, BodyFitParams& params) {
    std::string postprocesspts = params.outputPointsName(ptfilename);
    std::string bvhname = params.outputMotionName(ptfilename);
  
    LoadPoints(postprocesspts, _points);
    _model.init(bvhname);

    // init annotations
    Skeleton& skeleton = _model.skeleton;
    for (int i = 0; i < _model.motion.getNumKeys(); i++) {
      skeleton.setPose(_model.motion.getKey(i));
      vec3 neck = skeleton.getByName("neck")->getGlobalTranslation();
      vec3 torso = skeleton.getByName("torso")->getGlobalTranslation();
      vec3 lwrist = skeleton.getByName("lwrist")->getGlobalTranslation();
      vec3 rwrist = skeleton.getByName("rwrist")->getGlobalTranslation();
      vec3 threshold = 0.25f * neck + 0.75f * torso; 
      if (lwrist.y < threshold.y && rwrist.y < threshold.y) {
        annotations.push_back("NotSigning");
      }
      else {
        annotations.push_back("Signing");
      }
    }
  }

  void setup() {

    float contents[16] = {
        4.2845,   0.1087,   0.4054, 362.1306,
        0.2794,  -4.0146,  -0.4764, 656.7263,
        0.0,          0.0,       0.0,    0.0,
        0.0007,   0.0003,   0.0010,   1.0000
    };
    Cmatrix = transpose(glm::make_mat4(contents));

    _model.update(_current);
    _imageCurrent = 0; // force load on first frame

    _world = std::make_shared<dart::simulation::World>();
    _world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    _world->addSkeleton(_model.biped);

    IMGUI_CHECKVERSION();                                                          
    ImGui::CreateContext();                                                        
    ImPlot::CreateContext();                                                        
    ImGui::StyleColorsDark();                                                      
                                                                                 
#if defined(__APPLE__)                                                           
    // GL 3.2 + GLSL 150                                                           
    const char* glsl_version = "#version 150";                                     
#else                                                                            
    // GL 3.0 + GLSL 130                                                           
    const char* glsl_version = "#version 130";                                     
#endif                                                                           
    // Setup Platform/Renderer backends                                            
    ImGui_ImplGlfw_InitForOpenGL(window(), true);                                  
    ImGui_ImplOpenGL3_Init(glsl_version);                          
    
    // Set default parameters
    renderer.loadShader("floor", "../shaders/floor.vs", "../shaders/floor.fs");
    renderer.setUniform("MainTexture.tile", vec2(1.0));
    renderer.setUniform("Transparency", 1.0f);

    setWindowSize(WIDTH, HEIGHT);
    setup3D();
    loadImage();
  }

  vec3 project(const vec3& pos) {
    vec4 projected = Cmatrix * vec4( pos, 1.0);
    //std::cout << projected << std::endl;
    int sx = projected.x / projected.w;
    int sy = HEIGHT - projected.y / projected.w;
    return vec3(sx, sy, 0.0);
  }

  void drawEllipsoid(const glm::vec3& a, const glm::vec3& b, float radius) {
    vec3 direction = b - a; 
    float len = length(direction);
    if (len < radius) {
      drawSphere(a, radius);
      return;
    }

    vec3 scale = vec3(radius, radius, len);
    vec3 Z = direction/len;
    vec3 X = glm::vec3(1,0,0);
    vec3 Y = cross(Z, X); 
    X = cross(Y, Z);

    glm::mat3 rot(X, Y, Z);
    vec3 center = a + 0.5f * direction;
    atk::Transform TRS(glm::normalize(glm::quat(rot)), center, scale);

    renderer.push();
    renderer.transform(TRS.matrix());
    renderer.sphere();
    renderer.pop();
  }

  void drawCube(const glm::vec3& a, const glm::vec3& b, float radius) {
    vec3 direction = b - a; 
    
    float len = length(direction);
    if (len < radius) {
      renderer.push();
      renderer.translate(a);
      renderer.scale(vec3(radius));
      renderer.cube();
      renderer.pop();
      return;
    }

    vec3 scale = vec3(radius, radius, len);
    vec3 Z = direction/len;
    vec3 X = glm::vec3(1,0,0);
    vec3 Y = cross(Z, X); 
    X = cross(Y, Z);

    glm::mat3 rot(X, Y, Z);
    vec3 center = a + 0.5f * direction;
    atk::Transform TRS(glm::normalize(glm::quat(rot)), center, scale);

    renderer.push();
    renderer.transform(TRS.matrix());
    renderer.cube();
    renderer.pop();
  }

  void drawSphere(const vec3& pos, float radius) {
    renderer.push();
    renderer.translate(pos);
    renderer.scale(vec3(radius));
    renderer.sphere();
    renderer.pop();
  }

  void setColor(const vec3& color, bool isShaded) {
    vec3 ambient = isShaded? vec3(0.01f) : color;
    renderer.setUniform("Material.ambient", ambient);
    renderer.setUniform("Material.diffuse", color);
  }

  void drawSkeleton3D() {
    setColor(vec3(0.2f, 0.5f, 1.0f), true);

    float jointRadius = 0.05f;
    for (int i = 0; i < _model.skeleton.getNumJoints(); i++) {
      atk::Joint* joint = _model.skeleton.getByID(i);
      if (joint->getParent() == 0) continue;
      if (joint->getParent()->getName().find("Hand") != std::string::npos) continue; // skip hands

      glm::vec3 p1 = joint->getGlobalTranslation();
      glm::vec3 p2 = joint->getParent()->getGlobalTranslation();
      drawEllipsoid(p1, p2, jointRadius);
      drawSphere(p2, jointRadius);
      drawSphere(p1, jointRadius);
    }
  }

  void drawSkeleton2D() {

    renderer.push();
    float jointRadius = 5; // ASN TODO: Make a parameter
    for (int i = 0; i < _model.skeleton.getNumJoints(); i++) {
      atk::Joint* joint = _model.skeleton.getByID(i);
      if (joint->getParent() == 0) continue;
      if (joint->getParent()->getName().find("Hand") != std::string::npos) continue; // skip hands

      // TODO FIX: motion file is in centimeters but physicsModel is in meters
      // Need to compute a different C matrix!!
      glm::vec3 p1 = joint->getGlobalTranslation();
      glm::vec3 p2 = joint->getParent()->getGlobalTranslation();
      p1 = project(p1 * 100.0f);
      p2 = project(p2 * 100.0f);

      setColor(vec3(1,0,0), false);
      drawSphere(p1, jointRadius*2);

      if (joint->getName() == "head") {
        setColor(vec3(0,0,1), false);
        drawEllipsoid(p1, p2, jointRadius);
      } else if (joint->getName() == "lshoulder" || 
                 joint->getName() == "lelbow" ||
                 joint->getName() == "lwrist") {
        setColor(vec3(1,0,0), false);
        drawEllipsoid(p1, p2, jointRadius);

      } else if (joint->getName() == "rshoulder" || 
                 joint->getName() == "relbow" ||
                 joint->getName() == "rwrist") {
        setColor(vec3(0,1,0), false);
        drawEllipsoid(p1, p2, jointRadius);
      }
    }
    renderer.pop();
  }

  void drawPoints(bool is3D) {

    setColor(vec3(0), is3D);
    float jointRadius = is3D? 0.03 : 5; // ASN TODO: Make a parameter
    for (int i = 0; i < _points[_current].size(); i++) {
      vec3 p1 = _points[_current][i];
      if (!is3D) p1 = project(p1);
      else p1 = p1 * 0.01f;
      drawSphere(p1, jointRadius*2);

      /*
      vec2 p1 = points2D[_current][i];
      float yscale = 480/288.0; 
      float xscale = 720/384.0; 
      p1[0] = xscale * p1[0];
      p1[1] = HEIGHT - yscale * p1[1];
      drawSphere(vec3(p1[0], p1[1], 0), jointRadius*2);
      */
    }
  }

  void drawEntity(const dynamics::Entity* entity) {
    if (!entity) return;

    const auto& bodyNode = dynamic_cast<const dynamics::BodyNode*>(entity);
    if (bodyNode)
    {
      drawBodyNode(bodyNode);
      return;
    }

    const auto& shapeFrame = dynamic_cast<const dynamics::ShapeFrame*>(entity);
    if (shapeFrame)
    {
      drawShapeFrame(shapeFrame);
      return;
    }
  }

  void drawShapeFrame( const dynamics::ShapeFrame* shapeFrame) {
    if (!shapeFrame) return;

    const auto& visualAspect = shapeFrame->getVisualAspect();
    if (!visualAspect || visualAspect->isHidden()) return;

    mat4 trs = eigen2mat4(shapeFrame->getRelativeTransform()); 

    renderer.push(); 
    renderer.transform(transpose(trs));
    drawShape(shapeFrame->getShape().get());
    renderer.pop();
  }

  void drawShape(const dynamics::Shape* shape) {
    if (!shape) return;

    //std::cout << "drawShape " << std::endl;
    if (shape->is<dynamics::SphereShape>()) {
      const auto* sphere = static_cast<const dynamics::SphereShape*>(shape);
      //std::cout << "sphere: " << sphere->getRadius() << std::endl;
      renderer.push();
      renderer.scale(vec3(sphere->getRadius()));
      renderer.sphere();
      renderer.pop();
    }
    else if (shape->is<dynamics::EllipsoidShape>()) {
      const auto* ell = static_cast<const dynamics::EllipsoidShape*>(shape);
      Eigen::Vector3d size = ell->getDiameters();
      //std::cout << "ell: " << size << std::endl;
      renderer.push();
      renderer.scale(vec3(size(0), size(1), size(2)));
      renderer.sphere();
      renderer.pop();
    }
    else if (shape->is<dynamics::BoxShape>()) {
      const auto* box = static_cast<const dynamics::BoxShape*>(shape);
      Eigen::Vector3d size = box->getSize();
      //std::cout << "box: " << size(0) << " " << size(1) << " " << size(2) << " " << size << std::endl;
      renderer.push();
      renderer.scale(vec3(size(0), size(1), size(2)));
      renderer.cube();
      renderer.pop();
    }
  }

  void drawBodyNode(const dynamics::BodyNode* bodyNode) {

    if (!bodyNode) return;

    renderer.push();

    // Use the relative transform of this Frame. We assume that we are being
    // called from the parent Frame's renderer.
    mat4 trs = eigen2mat4(bodyNode->getRelativeTransform()); 
    renderer.transform(transpose(trs));

    auto shapeNodes = bodyNode->getShapeNodesWith<dynamics::VisualAspect>();
    for (const auto& shapeNode : shapeNodes)
      drawShapeFrame(shapeNode);

    for (const auto& entity : bodyNode->getChildEntities())
      drawEntity(entity);

    renderer.pop();
  }

  void drawPhysics() {

    setColor(vec3(1,1,0), true);
    for (auto i = 0u; i < _model.biped->getNumTrees(); ++i) {
      drawBodyNode(_model.biped->getRootBodyNode(i));
    }
    /*

    renderer.push();
    for (int i = 0; i < _model.skeleton.getNumJoints(); i++) {
      atk::Joint* joint = _model.skeleton.getByID(i);
      if (joint->getParent() == 0) continue;
      if (joint->getParent()->getName().find("Hand") != std::string::npos) continue; // skip hands

      glm::vec3 p1 = joint->getGlobalTranslation() ;
      glm::vec3 p2 = joint->getParent()->getGlobalTranslation() ;
      float jointRadius = _model.body.getRadius(joint->getName()) ;

      renderer.setUniform("Material.diffuse", vec3(1, 0, 0));
      drawSphere(p1, jointRadius*2);

      if (joint->getName() == "head") {
        renderer.setUniform("Material.diffuse", vec3(0, 0, 1));
        drawCube(p1, p2, jointRadius);
      } else if (joint->getName() == "lshoulder" || 
                  joint->getName() == "lelbow" ||
                joint->getName() == "lwrist") {
        renderer.setUniform("Material.diffuse", vec3(1, 0, 0));
        drawCube(p1, p2, jointRadius);

      } else if (joint->getName() == "rshoulder" || 
                  joint->getName() == "relbow" ||
                joint->getName() == "rwrist") {
        renderer.setUniform("Material.diffuse", vec3(0, 1, 0));
        drawCube(p1, p2, jointRadius);

      }
    }
    renderer.pop();
    */
  }

  void loadImage() {
    char buffer[64];
    std::snprintf(buffer, 64, "036CR1-%05d.png", _current+1);
    std::string filename = imagedir + buffer;
    _texture.load(filename);

    renderer.loadTexture("video", _texture, 1);  
    renderer.setUniform("MainTexture.enabled", true);
    renderer.texture("MainTexture.texture", "video");

    _imageCurrent = _current;
  }

  void draw() {
    if (_paused) {
      _time = _current * _model.motion.getDeltaTime();

    } else {
      _time += dt() * _timeScale;
      float u = fmod(_time, _model.motion.getDuration()) / _model.motion.getDuration();
      _current = (int)(u * _model.motion.getNumKeys());
    }
    _model.update(_current);

    if (_3d) {
      if (_showPhysics) {
        drawPhysics();
      }
      if (_showSkeleton) {
        drawSkeleton3D();
      }
      if (_showPoints) {
        drawPoints(true);
      }
      drawFloor(20, 2, 0.5);

    } 
    else {

      if (_current != _imageCurrent) {
        loadImage();
      }
      
      renderer.blendMode(agl::BLEND);
      renderer.setUniform("MainTexture.enabled", true);
      renderer.texture("MainTexture.texture", "video");
      renderer.setUniform("Material.ambient", vec3(0.2f));
      renderer.setUniform("Material.diffuse", vec3(1));
      renderer.setUniform("Material.specular", vec3(0));
      renderer.setUniform("Transparency", 1.0f);
      renderer.push();
      renderer.translate(vec3(WIDTH*0.5f, HEIGHT*0.5f, -900.0f));
      renderer.scale(vec3(WIDTH, HEIGHT, 1.0));
      renderer.rotate(kPI/2, vec3(1, 0, 0));
      renderer.plane();
      renderer.pop();

      renderer.setUniform("MainTexture.enabled", false);
      renderer.setUniform("Transparency", 1.0f);
      if (_showSkeleton) {
        drawSkeleton2D();
      }
      if (_showPoints) {
        drawPoints(false);
      }
    }

    renderer.text(_paused ? "Paused" : "Playing", 10, 15);
    renderer.text("Current frame: " + std::to_string(_current), 10, 35);
    renderer.text("Time scale: " + std::to_string(_timeScale), 10, 55);
    renderer.text(annotations[_current], 10, 75);
  }

  void keyUp(int key, int mods) {
    if (_3d && key == 'M') { 
      _showPhysics = !_showPhysics; 

    } else if (key == 'P') { 
      _showPoints = !_showPoints; 
    
    } else if (key == '3') { 
      _3d = !_3d; 
      if (_3d) {
        setup3D();
      } else {
        setupVideo();
      }
    } else if (key == ' ') {
      _paused = !_paused;
    }
    else if (_paused && key == GLFW_KEY_UP) {
      _current = (_current + 1) % _model.motion.getNumKeys();
    }
    else if (_paused && key == GLFW_KEY_DOWN) {
      _current = _current - 1;
      if (_current < 0)
        _current = _model.motion.getNumKeys() - 1;
    }
    else if (key == '[') {
      _timeScale = std::max<float>(_timeScale - 0.1, 0.1);
    }
    else if (key == ']') {
      _timeScale = std::min<float>(_timeScale + 0.1, 10.0);
    }
    else if (_3d && key == 'H') {
      vec3 pos = _model.skeleton.getRoot()->getGlobalTranslation();
      lookAt(pos + vec3(.5f, 1.0f, .5f), pos);
    }
    else if (_3d && key == 'W') {
      vec3 pos = _model.skeleton.getByName("rwrist")->getGlobalTranslation();
      lookAt(pos + vec3(.5f, 1.0f, .5f), pos);
    }
    else if (key == 'S') {
      _showSkeleton = !_showSkeleton;
    }
  }

  void mouseDown(int button, int mods) {
    vec2 pos = mousePosition();
    pos.y = HEIGHT - pos.y;
    std::cout << glm::to_string(pos) << std::endl;
  }

  void setupVideo() {
    setCameraEnabled(false);
    lookAt(vec3(0.0), vec3(0, 0, -2));
    ortho(0, WIDTH, 0, HEIGHT, -1000, 1000);
    background(vec3(0.0f));
  }

  void setup3D() {
    setCameraEnabled(true);
    camera.setMoveSpeed(0.1);
    background(vec3(0.8));
    vec3 center = vec3(0, 0, 0);
    vec3 dim = vec3(5, 10, 5);
    setupPerspectiveScene(center, dim);
    vec3 pos = _model.skeleton.getRoot()->getGlobalTranslation();
    lookAt(pos + vec3(0.95, 0.1, 0.95), vec3(0,1,0));
  }

  void drawFloor(float size, float big, float small) {
    renderer.beginShader("floor");
    renderer.setUniform("uFog.color", vec3(0.8));
    renderer.setUniform("uLargeBlockSize", big);
    renderer.setUniform("uSmallBlockSize", small);
    renderer.setUniform("uFog.minDist", 0.75f * size);
    renderer.setUniform("uFog.maxDist", size);
    renderer.push();
    renderer.scale(vec3(size, 1, size));
    renderer.plane();
    renderer.pop();
    renderer.endShader();
  }

 private:
  agl::Image _texture;
  int _current = 0;
  int _imageCurrent = 0;
  bool _paused = true;
  bool _showPhysics = false;
  bool _showSkeleton = true;
  bool _showPoints = true;
  bool _3d = true;
  float _time = 0;
  float _timeScale = 1;
  Eigen::MatrixXd C;
  dart::simulation::WorldPtr _world;
  std::vector<std::vector<vec3>> _points;
  DartSkeleton _model;
};

int main(int argc, char** argv)
{
  std::string inifilename = "../demo/skeleton.ini";
  std::string ptfilename = "../demo/036CR1-3d.csv";

  bool showWindow = true;
  bool showHelp = false;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--no-win") {
      showWindow = false;

    } else if (arg == "-h" || arg == "--help") {
      showHelp = true;

    } else {
      ptfilename = arg;
    }
  } 

  if (ptfilename == "" || showHelp) {
    std::cout << "usage: open-body-fit [--no-win] points.csv\n";
    return 0;
  }

  BodyFitParams params;
  if (!params.load(inifilename)) {
    std::cout << "Can't load " << inifilename << std::endl;
    return false;
  }

  LevmarSolve(ptfilename, params);
  DartSkeleton model;
  model.init(params.outputMotionName(ptfilename));
  model.save(params.outputPrefix(ptfilename));

  if (showWindow) {
    VideoViewer viewer;
    viewer.load(ptfilename, params);
    viewer.run();
  }
}
