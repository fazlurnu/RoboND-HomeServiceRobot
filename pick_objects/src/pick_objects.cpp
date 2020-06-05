#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool readParameters(ros::NodeHandle nodeHandle, float& pick_up_x, float& pick_up_y, float& pick_up_w, float& drop_off_x, float& drop_off_y, float& drop_off_w){
	if(!nodeHandle.getParam("/pick_objects/pick_up_x", pick_up_x)){
		return false;
	}
	if(!nodeHandle.getParam("/pick_objects/pick_up_y", pick_up_y)){
		return false;
	}
	if(!nodeHandle.getParam("/pick_objects/pick_up_w", pick_up_w)){
		return false;
	}
	if(!nodeHandle.getParam("/pick_objects/drop_off_x", drop_off_x)){
		return false;
	}
	if(!nodeHandle.getParam("/pick_objects/drop_off_y", drop_off_y)){
		return false;
	}
	if(!nodeHandle.getParam("/pick_objects/drop_off_w", drop_off_w)){
		return false;
	}
	return true;
}
/*
float static ;
float static pick_up_y;
float static pick_up_w;
float static drop_off_x;
float static drop_off_y;
float static drop_off_w;
*/
int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");  
  ros::NodeHandle nodeHandle("~");

  float pick_up_x = 0.0;
  float pick_up_y = 0.0;
  float pick_up_w = 0.0;
  float drop_off_x = 1.0;
  float drop_off_y = 1.0;
  float drop_off_w = 1.0;

  if (!readParameters(nodeHandle, pick_up_x, pick_up_y, pick_up_w, drop_off_x, drop_off_y, drop_off_w)){
		ROS_ERROR("Could not read params.");
		ros::requestShutdown();
	}

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  int nb_goals = 2;
  int nb_states = 3; // state: x, y, w
  float goals[nb_goals][nb_states] = { {pick_up_x, pick_up_y, pick_up_w}, {drop_off_x, drop_off_y, drop_off_w}  };

  for (int i=0; i < nb_goals; i++){

      goal.target_pose.pose.position.x = goals[i][0];
      goal.target_pose.pose.position.y = goals[i][1];
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goals[i][2]);

       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending goal [%f], [%f], [%f]", goals[i][0], goals[i][1], goals[i][2]);
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      // Check if the robot reached its goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached!");
      else
        ROS_INFO("Goal cannot be reached!");
      ros::Duration(5.0).sleep();
  }

  return 0;
}