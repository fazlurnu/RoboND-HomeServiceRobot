#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle nodeHandle("~");
  ros::Rate r(1);
  ros::Publisher marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  float pick_up_x = -2.5;
  float pick_up_y = 3.0;
  float pick_up_w = 0.0;
  float drop_off_x = 2.5;
  float drop_off_y = -3.0;
  float drop_off_w = 1.5707;

  // Define a position and orientation for the robot to reach
  int nb_goals = 2;
  int nb_states = 3; // state: x, y, w
  float goals[nb_goals][nb_states] = { {pick_up_x, pick_up_y, pick_up_w}, {drop_off_x, drop_off_y, drop_off_w}  };

  // Set our initial shape type to be a sphere
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  visualization_msgs::Marker marker;
    
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = goals[0][0];
  marker.pose.position.y = goals[0][1];
  marker.pose.position.z = 0.5;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(goals[0][2]);

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  marker_pub.publish(marker);
  ROS_INFO("Pick Up Marker at [%f], [%f], [%f]", goals[0][0], goals[0][1], goals[0][2]);
  
  ros::Duration(5.0).sleep();

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Pick Up Done");

  ros::Duration(5.0).sleep();
  
  marker.pose.position.x = goals[1][0];
  marker.pose.position.y = goals[1][1];
  marker.pose.position.z = 0.5;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(goals[1][2]);
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Drop Off Marker at [%f], [%f], [%f]", goals[1][0], goals[1][1], goals[1][2]);
 
  r.sleep();
}
