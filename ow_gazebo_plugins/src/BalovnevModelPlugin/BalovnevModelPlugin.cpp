// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#define _USE_MATH_DEFINES
#include <cmath>
#include <gazebo/physics/Link.hh>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float64.h>

#include <BalovnevModelPlugin.h>

using namespace ros;
using namespace gazebo;
using namespace ow_dynamic_terrain;
using namespace cv_bridge;
using namespace std;
using namespace ignition::math;

// world properties
const double GRAVITY                  = 1.315;           // m/s^2

// scoop properties
const double BUCKET_WIDTH             = 0.06;            // m
const double BUCKET_SIDE_PLATE_LENGTH = 0.08;            // m
const double BUCKET_HEIGHT_FROM_TIP   = 0.06;            // m
const double BLUNT_EDGE_THICK         = 0.002;           // m
const double BLUNT_EDGE_ANGLE         = 45./180 * M_PI;  // rad
const double SIDE_PLATE_THICK         = 0.0016;          // m
const double BLADE_RADIUS             = 0.2;             // m

// regolith properties
const double SOIL_DENSITY             = 1700;            // kg/m^3
const double COHESION                 = 1500;            // N/m^2
const double INT_FRICTION_ANGLE       = 44./180 * M_PI;  // rad
const double EXT_FRICTION_ANGLE       = 28./180 * M_PI;  // rad
const double SURCHARGE_MASS           = 1;               // kg/m^2
const double SOIL_PRISM_HEIGHT        = 0;               // m


// real-time variables placeheld by constants
const double RAKE_ANGLE               = 10./180 * M_PI;  // rad
const double BUCKET_VELOCITY          = 0.1;             // m/s
const int BURIED = 0;  // define BURIED: BURIED = 1 if entire bucket is below the soil otherwise BURIED = 0 

// constants specific to the scoop end-effector
const string SCOOP_LINK_NAME       = "lander::l_scoop";

const string TOPIC_MODIFY_TERRAIN_VISUAL = "/ow_dynamic_terrain/modification_differential/visual";
const string TOPIC_BALOVNEV_HORIZONTAL   = "/balovnev_model/horizontal_force";
const string TOPIC_BALOVNEV_VERTICAL     = "/balovnev_model/vertical_force";

const Duration DIG_TIMEOUT_INTERVAL = Duration(1.0); // seconds

// define sin-squared function for use in getParameterA
static double sin2(double x) {
  double y = sin(x);
  return y * y;
}

void BalovnevModelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  m_node_handle = make_unique<ros::NodeHandle>(sdf->GetName());
  
  m_link = model->GetLink(SCOOP_LINK_NAME);
  if(!m_link) {
    gzerr << "Load - specified link is invalid." << endl;
    return;
  }
  m_sub_mod_diff_visual = m_node_handle->subscribe(
    TOPIC_MODIFY_TERRAIN_VISUAL, 1,
    &BalovnevModelPlugin::onModDiffVisualMsg, this
  );

  m_pub_horizontal_force = m_node_handle->advertise<std_msgs::Float64>(
    TOPIC_BALOVNEV_HORIZONTAL, 1, true
  );
  m_pub_vertical_force = m_node_handle->advertise<std_msgs::Float64>(
    TOPIC_BALOVNEV_VERTICAL, 1, true
  );

  m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(
    std::bind(&BalovnevModelPlugin::onUpdate, this)
  );

  resetForces();

  m_dig_timeout = m_node_handle->createTimer(
    DIG_TIMEOUT_INTERVAL, &BalovnevModelPlugin::onDigTimeout, this, true, false
  );

  gzlog << "BavlovnevModelPlugin - successfully loaded" <<endl;
}


void BalovnevModelPlugin::onUpdate()
{
  if(!m_link) {
    gzwarn << " m_link is invalid." << endl;
    return;
  }
  m_link->AddRelativeForce(
    ignition::math::Vector3d(-m_horizontal_force, 0, m_vertical_force)
  );
}

// Calculate the parameter A
// x   - angle
// ifa - int friction angle
// efa - ext friction angle
double BalovnevModelPlugin::getParameterA(double x, double ifa, double efa)
{  
  if (x <= 0.5 * asin(sin(efa)/sin(ifa)) - efa)
  {
    double a = (1 - sin(ifa) * cos(2 * x)) / (1 - sin(ifa));
    return a;
  } 
  else 
  {
    double a = (cos(efa) * (cos(efa) + sqrt(sin2(ifa) - sin2(efa)))
        / (1 - sin(ifa))) * exp((2 * x - M_PI + efa + asin(sin(efa)
        / sin(ifa))) * tan(ifa));
    return a;
  }
}

// calculate the force
void BalovnevModelPlugin::computeForces(double vertical_cut_depth)
{
  double g    = GRAVITY;
  double beta = RAKE_ANGLE;
  double w    = BUCKET_WIDTH;
  double ls   = BUCKET_SIDE_PLATE_LENGTH;
  double l    = BUCKET_HEIGHT_FROM_TIP;
  double et   = BLUNT_EDGE_THICK;
  double ea   = BLUNT_EDGE_ANGLE;
  double s    = SIDE_PLATE_THICK;
  double r    = BLADE_RADIUS;
  double sd   = SOIL_DENSITY;
  double c    = COHESION;
  double phi  = INT_FRICTION_ANGLE;
  double delta= EXT_FRICTION_ANGLE;
  double q    = SURCHARGE_MASS; 
  double h    = SOIL_PRISM_HEIGHT;
  double v    = BUCKET_VELOCITY;
  // define vertical_cut_depth
  double d = vertical_cut_depth;
  // get constant a1,a2,a3
  double a1 = getParameterA(beta, phi, delta);
  double a2 = getParameterA(ea, phi, delta);
  double a3 = getParameterA(M_PI_2, phi, delta);
  //calculate horizontal force and vertical force
  m_horizontal_force =
    w*d*a1 * (1+(1/tan(beta))*tan(delta)) * (d*g*sd/2 + c*(1/tan(phi))
    + g*q + BURIED * (d-l*sin(beta)) * g*sd * (1-sin(phi)) / (1+sin(phi)))
    + w*et*a2 * (1 + tan(delta)*(1/tan(ea))) * (et*g*sd/2 + c*(1/tan(phi))
    + g*q + d*g*sd * (1-sin(phi))/(1+sin(phi)))
    + d*a3 * (2*s + 4*ls*tan(delta))*(d*g*sd/2 + c*(1/tan(phi))
    + g*q + BURIED * (d-ls*sin(beta)) * g*sd * (1-sin(phi))/(1+sin(phi)));

  m_vertical_force = m_horizontal_force * cos(beta+delta) / sin(beta+delta);

  publishForces();

  // DEBUG CODE
  // gzlog << "horizontal_force" << m_horizontal_force <<"   vertical_force" << m_vertical_force << endl;
}

void BalovnevModelPlugin::publishForces() {
  static std_msgs::Float64 hf, vf;
  hf.data = m_horizontal_force;
  vf.data = m_vertical_force;
  m_pub_horizontal_force.publish<std_msgs::Float64>(hf);
  m_pub_vertical_force.publish<std_msgs::Float64>(vf);
}

void BalovnevModelPlugin::resetForces() {
  m_vertical_force = 0.0;
  m_horizontal_force = 0.0;
  publishForces();
}

bool BalovnevModelPlugin::isScoopDigging() {
  static const Vector3 SCOOP_DOWNWARD(0.0, 0.0, 1.0);
  static const Vector3 WORLD_DOWNWARD(0.0, 0.0, -1.0);
  Vector3 scoop_bottom(
    m_link->WorldPose().Rot().RotateVector(SCOOP_DOWNWARD)
  );
  return WORLD_DOWNWARD.Dot(scoop_bottom) > 0.0;
}

void BalovnevModelPlugin::onModDiffVisualMsg(
  const modified_terrain_diff::ConstPtr &msg)
{
  // ignore message if it wasn't generated by a valid scoop dig angle
  if (!isScoopDigging())
    return;

  // import image to so we can traverse it
  auto image_handle = CvImageConstPtr();
  try {
    image_handle = toCvShare(msg->diff, msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  auto rows = image_handle->image.rows;
  auto cols = image_handle->image.cols;
  if (rows <= 0 || cols <= 0) {
    ROS_WARN("Differential image dimensions are zero or negative");
    return;
  }

  // get average height
  float sum = 0.0;
  for (auto y = 0; y < rows; ++y) {
    for (auto x = 0; x < cols; ++x) {
      // NOTE: pixel values should always be negative
      sum += -image_handle->image.at<float>(y, x);
    }
  }
  float depth = sum / (rows * cols);

  computeForces(depth);

  // reset timeout at each terrain modification
  m_dig_timeout.stop();
  m_dig_timeout.start();
}

void BalovnevModelPlugin::onDigTimeout(const TimerEvent &) {
  resetForces();
}
