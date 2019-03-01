#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


using namespace std;


void makeTransform(double x, double y, double z, const string& child_frame,
                   geometry_msgs::TransformStamped& xform)
{
  xform.transform.translation.x = x;
  xform.transform.translation.y = y;
  xform.transform.translation.z = z;

  xform.transform.rotation.x = 0;
  xform.transform.rotation.y = 0;
  xform.transform.rotation.z = 0;
  xform.transform.rotation.w = 1;

  xform.header.frame_id = "site"; // This is the parent in the node tree
  xform.child_frame_id = child_frame;
}

int main(int argc, char* argv[])
{
  // Set up ROS
  ros::init(argc, argv, "ephemeris_node");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster broadcaster;

  bool first_error = true;
  ros::Rate rate(1);
  while (ros::ok())
  {
    // Get the current time and make sure it is reasonable.    
    ros::Time current_time = ros::Time::now();
    if (current_time.sec < 400000000) // An arbitrary time very far from 0! 
    {
      if (!first_error) // This always happens once in sim so suppress the first error message.
        ROS_ERROR_STREAM("Got low value for ros::Time::now() in the solar frame publisher: "
                         << current_time);
      first_error = false;
      sleep(3); // Wait to see if a valid time is published
      continue;
    }

    // Publish the transforms we are interested in.
    // Using test data right now because Spice hasn't been integrated.
    geometry_msgs::TransformStamped xformStamped;
    makeTransform(551000000000, 40000000000, 551000000000, "sun", xformStamped);
    broadcaster.sendTransform(xformStamped);
    makeTransform(-450000000, 0, 100000000, "jupiter", xformStamped);
    broadcaster.sendTransform(xformStamped);

    ros::spinOnce();
    rate.sleep();
  }
}

