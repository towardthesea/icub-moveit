/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "moveit_msgs/MoveGroupActionResult.h"

#include "eigen3/Eigen/src/Geometry/Transform.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yarp_msg/object.h>
#include <yarp_msg/SharedData_new.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>
#include <iostream>
#include <vector>
#include <stdarg.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>


//#include "object3D.h"

using namespace std;
using namespace moveit::core;
using namespace trajectory_msgs;

//vector<object3D> obstacles;	// Remember to clear it after passing to environment
vector<yarp_msg::object> obsSet;
yarp_msg::object target;
bool envChanged = false;
ros::Publisher pub_signal2Yarp;

moveit_msgs::MoveGroupActionResult 	moveitResult;
vector<double> 						jointValues;
Eigen::Affine3d 					end_effector_state;
Eigen::Affine3d 					elbow_state;
JointTrajectory 					simpleTrajectory;
vector<double>						planningTime_all;
vector<double> 						costEE_all, costElbow_all;
double								numberObstacles;
//RobotState 							mrobotState;
//robot_model::RobotModelPtr 			robot_model;

const string groupName = "left_arm_torso";
const string endeffectorName = "left_palm";
const char *fileName = "/home/pnguyen/ROS_workspace/src/icub-moveit/batchSummaryROS.txt";

template <typename T>
string NumberToString ( T Number )
{
    stringstream ss;
    ss << Number;
    return ss.str();
}

void resultCallback(const moveit_msgs::MoveGroupActionResult& msg)
{
	simpleTrajectory.points.clear();

	moveitResult = msg;

	ROS_INFO("I heard: [%d]", (int)msg.result.planned_trajectory.joint_trajectory.points.size());
	for(int i = 0; i < msg.result.planned_trajectory.joint_trajectory.points.size(); i++)
	{
		JointTrajectoryPoint auxPoint;

		for(int j = 0; j < msg.result.planned_trajectory.joint_trajectory.points[i].positions.size(); j++)
		{
			auxPoint.positions.push_back(msg.result.planned_trajectory.joint_trajectory.points[i].positions[j]);

		}
		simpleTrajectory.points.push_back(auxPoint);

	}

	planningTime_all.push_back(msg.result.planning_time);

//	RobotState cRobotState()
}

void objectsCallback(const yarp_msg::object& object)
{
	ROS_INFO("Received an object");
	if (object.objectType == "obstacle")
	{
//		object3D obs;
		ROS_INFO("Received an obstacle");
		obsSet.push_back(object);
	}
	else if (object.objectType == "target")
	{
		ROS_INFO("Received an obstacle");
		target = object;
	}

	envChanged = true;

//	if (obsSet.size()==numberObstacles)
//		envChanged = true;
//	else
//		envChanged = false;
}

void numberObstaclesCallback(const yarp_msg::SharedData_new& msg)
{
	if (msg.text == "obstacles")
	{
		numberObstacles = msg.content[0];
		ROS_INFO("Number of obstacles: %d", (int)numberObstacles);
	}
}

void addValueTrajCart(const Eigen::Affine3d& mState, vector<geometry_msgs::Pose>& trajCart);
double distancePoint2Point(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
double computeMotionDistance(vector<geometry_msgs::Pose>& trajCart);

void initBatchSummary();
void logBatchSummary(const int& count);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;

	ros::Subscriber sub_result = node_handle.subscribe("/move_group/result", 1, resultCallback);
	ros::Subscriber sub = node_handle.subscribe("/yarp_connector/objects", 1, objectsCallback);
	ros::Subscriber sub_nbObstacles = node_handle.subscribe("/yarp_connector/data", 1, numberObstaclesCallback);
	pub_signal2Yarp = node_handle.advertise<yarp_msg::SharedData_new>("signal2Yarp",1);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	srand (time(NULL));


	/* This sleep is ONLY to allow Rviz to come up */
	sleep(10.0);

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	//robot_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
	kinematic_state->setToDefaultValues(kinematic_state->getJointModelGroup(groupName), "home_table_left_torso");
	//  kinematic_state->setToDefaultValues(kinematic_state->getJointModelGroup("left_arm"), "home_table_left");

	const robot_state::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(groupName);
	//  const robot_state::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("left_arm");

	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	// Get Joint Values
	// ^^^^^^^^^^^^^^^^
	// We can retreive the current set of joint values stored in the state for the right arm.
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	RobotState mrobotState(robot_model);
//	robot_state = RobotState(robot_model);
//	mrobotState.RobotState(robot_model);

	mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
	//  robot_state.setToDefaultValues(robot_state.getJointModelGroup("left_arm"), "home_table_left");


//	kinematic_state->computeCartesianPath(joint_model_group,moveitResult.result.planned_trajectory,)
	mrobotState.update("true");

//	const Eigen::Affine3d &end_effector_state = robot_state.getGlobalLinkTransform("left_palm");
	end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");

	/* Print end-effector pose. Remember that this is in the model frame */
	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());




	// BEGIN_TUTORIAL
	//
	// Setup
	// ^^^^^
	//
	// The :move_group_interface:`MoveGroup` class can be easily
	// setup using just the name
	// of the group you would like to control and plan for.
	moveit::planning_interface::MoveGroup group(groupName);
	//  moveit::planning_interface::MoveGroup group("left_arm");

	//  moveit::planning_interface::MoveGroup group("left_arm", "robot_description", robot_model);

	joint_values = group.getCurrentJointValues();
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}


	group.setStartState(mrobotState);
	group.setPlannerId("RRTstarkConfigDefault");
	group.setPlanningTime(1.0);
	//  group.setNumPlanningAttempts(1);
	group.setGoalPositionTolerance(0.05);
	group.setGoalOrientationTolerance(5.0);


	sleep(1.0);

	//  joint_values = group.getCurrentJointValues();
	mrobotState.copyJointGroupPositions(joint_model_group, joint_values);
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("End-effector frame: %s", group.getEndEffectorLink().c_str());

	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the
	// end-effector.
	ROS_INFO("=========================================================");
	ROS_INFO("Pose goal planning");
	ROS_INFO("=========================================================");
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	//  target_pose1.position.x = -0.25;
	//  target_pose1.position.y =  0.0;
	//  target_pose1.position.z =  0.15;
	target_pose1.position.x = -0.4;
	target_pose1.position.y =  0.1;
	target_pose1.position.z =  0.1;
	group.setPoseTarget(target_pose1,"left_palm");


	// Now, we call the planner to compute the plan
	// and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success;


	/*bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	 Sleep to give Rviz time to visualize the plan.
	sleep(2.0);

	if (success)
	{
		 Print end-effector pose. Remember that this is in the model frame
		vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
		for (int i=0; i<simpleTrajectory.points.size(); i++)
		{
			mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
			end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
			elbow_state = mrobotState.getGlobalLinkTransform("la4");
			addValueTrajCart(end_effector_state,trajCartEE);
			addValueTrajCart(elbow_state,trajCartElbow);

		}
		double costEE = computeMotionDistance(trajCartEE);
		double costElbow = computeMotionDistance(trajCartElbow);

		ROS_INFO("cost EE= %f",costEE);
		ROS_INFO("cost Elbow= %f",costElbow);
	}


	// Visualizing plans
	// ^^^^^^^^^^^^^^^^^
	// Now that we have a plan we can visualize it in Rviz.  This is not
	// necessary because the group.plan() call we made above did this
	// automatically.  But explicitly publishing plans is useful in cases that we
	// want to visualize a previously created plan.
	if (success)
	{
		ROS_INFO("Visualizing plan 1 (again)");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);

		 Sleep to give Rviz time to visualize the plan.
		sleep(2.0);
	}*/

	// Moving to a pose goal
	// ^^^^^^^^^^^^^^^^^^^^^
	//
	// Moving to a pose goal is similar to the step above
	// except we now use the move() function. Note that
	// the pose goal we had set earlier is still active
	// and so the robot will try to move to that goal. We will
	// not use that function in this tutorial since it is
	// a blocking function and requires a controller to be active
	// and report success on execution of a trajectory.

	/* Uncomment below line when working with a real robot*/
	/* group.move() */

	/*  // Planning to a joint-space goal
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Let's set a joint space goal and move towards it.  This will replace the
	// pose target we set above.
	//
	// First get the current set of joint values for the group.
	ROS_INFO("=========================================================");
	ROS_INFO("Joint goal planning");
	ROS_INFO("=========================================================");
	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	// Now, let's modify one of the joints, plan to the new joint
	// space goal and visualize the plan.
	group_variable_values[0] = -1.0;
	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
	Sleep to give Rviz time to visualize the plan.
	sleep(5.0);*/

	/*  // Planning with Path Constraints
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Path constraints can easily be specified for a link on the robot.
	// Let's specify a path constraint and a pose goal for our group.
	// First define the path constraint.
	ROS_INFO("=========================================================");
	ROS_INFO("Pose goal planning with path constraints");
	ROS_INFO("=========================================================");
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "left_palm";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	group.setPathConstraints(test_constraints);

	// We will reuse the old goal that we had and plan to it.
	// Note that this will only work if the current state already
	// satisfies the path constraints. So, we need to set the start
	// state to a new pose.
	robot_state::RobotState start_state(*group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = -0.4;
	start_pose2.position.y = -0.05;
	start_pose2.position.z =  0.3;
	const robot_state::JointModelGroup *joint_model_group =
				  start_state.getJointModelGroup(group.getName());
	start_state.setFromIK(joint_model_group, start_pose2);
	group.setStartState(start_state);

	// Now we will plan to the earlier pose target from the new
	// start state that we have just created.
	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
	Sleep to give Rviz time to visualize the plan.
	sleep(10.0);

	// When done with the path constraint be sure to clear it.
	group.clearPathConstraints();*/

	/*  // Cartesian Paths
	// ^^^^^^^^^^^^^^^
	// You can plan a cartesian path directly by specifying a list of waypoints
	// for the end-effector to go through. Note that we are starting
	// from the new start state above.  The initial pose (start state) does not
	// need to be added to the waypoint list.
	std::vector<geometry_msgs::Pose> waypoints;

	geometry_msgs::Pose target_pose3 = start_pose2;
	target_pose3.position.x += 0.2;
	target_pose3.position.z += 0.2;
	waypoints.push_back(target_pose3);  // up and out

	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);  // left

	target_pose3.position.z -= 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3);  // down and right (back to start)

	// We want the cartesian path to be interpolated at a resolution of 1 cm
	// which is why we will specify 0.01 as the max step in cartesian
	// translation.  We will specify the jump threshold as 0.0, effectively
	// disabling it.
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(waypoints,
											   0.01,  // eef_step
											   0.0,   // jump_threshold
											   trajectory);

	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
		fraction * 100.0);
	Sleep to give Rviz time to visualize the plan.
	sleep(15.0);*/

	int countReplan = 0;
	initBatchSummary();

	ROS_INFO("While loop to wait for the obstacle from YARP");
	while (true)
	{
		if (envChanged)
		{
//			vector<double> costEE_all, costElbow_all;
			countReplan++;

			sleep(3.0);
			envChanged = false;
			// Adding/Removing Objects and Attaching/Detaching Objects
			// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			ROS_INFO("=========================================================");
			ROS_INFO("Setup environment");
			ROS_INFO("=========================================================");
			// First, we will define the collision object message.
			moveit_msgs::CollisionObject collision_object;
			collision_object.header.frame_id = group.getPlanningFrame();

			/* The id of the object is used to identify it. */
			collision_object.id = "box0";

			/* Define a box to add to the world. */
			shape_msgs::SolidPrimitive primitive;
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
		//	primitive.dimensions[0] = 0.1;
		//	primitive.dimensions[1] = 0.1;
		//	primitive.dimensions[2] = 0.1;

			/* A pose for the box (specified relative to frame_id) */
			geometry_msgs::Pose box_pose;
			box_pose.orientation.w = 1.0;
		//	box_pose.position.x = -0.2;
		//	box_pose.position.y = -0.2;
		//	box_pose.position.z =  0.1;

		//	collision_object.primitives.push_back(primitive);
		//	collision_object.primitive_poses.push_back(box_pose);
		//	collision_object.operation = collision_object.ADD;

			std::vector<moveit_msgs::CollisionObject> collision_objects;
			//  collision_objects.push_back(collision_object);

			//  // Now, let's add the collision object into the world
			//  ROS_INFO("Add an object into the world");
			//  planning_scene_interface.addCollisionObjects(collision_objects);

			std::vector<string> object_ids;
			// More obstacle boxes
			ROS_INFO("Number of obstacle in obsSet: %d", obsSet.size());
			for (int i=0; i<obsSet.size(); i++)
			{
			/* The id of the object is used to identify it. */
				string boxid = "box" + NumberToString(i);
				collision_object.id = boxid;
				object_ids.push_back(boxid);

				primitive.dimensions.resize(3);
				//	  primitive.dimensions[0] = 0.1;
				//	  primitive.dimensions[1] = 0.1;
				//	  primitive.dimensions[2] = 0.1;
				primitive.dimensions[0] = obsSet[i].dimension[0];
				primitive.dimensions[1] = obsSet[i].dimension[1];
				primitive.dimensions[2] = obsSet[i].dimension[2];

				box_pose.position.x = obsSet[i].position[0];
				box_pose.position.y = obsSet[i].position[1];
				box_pose.position.z = obsSet[i].position[2];

				collision_object.primitives.push_back(primitive);
				collision_object.primitive_poses.push_back(box_pose);
				collision_object.operation = collision_object.ADD;

				//	  std::vector<moveit_msgs::CollisionObject> collision_objects;
				collision_objects.push_back(collision_object);

				// Now, let's add the collision object into the world
				ROS_INFO("Add an object into the world");

		/*		box_pose.position.x = (rand()%10 - 5)/10.0;
				box_pose.position.y = (rand()%10 - 5)/10.0;
				box_pose.position.z = (rand()%10 - 2)/10.0;

				if ((abs(target_pose1.position.x-box_pose.position.x) > primitive.dimensions[0]/2.0) &&
					  (abs(target_pose1.position.y-box_pose.position.y) > primitive.dimensions[1]/2.0) &&
					  (abs(target_pose1.position.z-box_pose.position.z) > primitive.dimensions[2]/2.0))
				{

					collision_object.primitives.push_back(primitive);
					collision_object.primitive_poses.push_back(box_pose);
					collision_object.operation = collision_object.ADD;

					//	  std::vector<moveit_msgs::CollisionObject> collision_objects;
					collision_objects.push_back(collision_object);
				}*/


			}

//			obsSet.clear();


			planning_scene_interface.addCollisionObjects(collision_objects);

			/* Sleep so we have time to see the object in RViz */
			sleep(5.0);

			// Planning with collision detection can be slow.  Lets set the planning time
			// to be sure the planner has enough time to plan around the box.  10 seconds
			// should be plenty.
			ROS_INFO("Waiting for planning...");
			group.setPlanningTime(10.0);


			// Now when we plan a trajectory it will avoid the obstacle
			//  group.setStartState(*group.getCurrentState());
			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);


			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with RRTstar");
			ROS_INFO("=========================================================");
			group.setPlannerId("RRTstarkConfigDefault");
			success = group.plan(my_plan);

			ROS_INFO("Visualizing plan 2 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with RRTConnect");
			ROS_INFO("=========================================================");
			group.setPlannerId("RRTConnectkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 3 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			//  ROS_INFO("=========================================================");
			//  ROS_INFO("Pose goal planning with obstacle with TRRT");
			//  ROS_INFO("=========================================================");
			//  group.setPlannerId("TRRTkConfigDefault");
			//  success = group.plan(my_plan);
			//  ROS_INFO("Visualizing plan 4 (pose goal move around box) %s",
			//    success?"":"FAILED");
			//  /* Sleep to give Rviz time to visualize the plan. */
			//  sleep(5.0);

			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with PRMstar");
			ROS_INFO("=========================================================");
			group.setPlannerId("PRMstarkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with SBL");
			ROS_INFO("=========================================================");
			group.setPlannerId("SBLkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 6 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with EST");
			ROS_INFO("=========================================================");
			group.setPlannerId("ESTkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 7 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with LBKPIECE");
			ROS_INFO("=========================================================");
			group.setPlannerId("LBKPIECEkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 8 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with BKPIECE");
			ROS_INFO("=========================================================");
			group.setPlannerId("BKPIECEkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 9 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}


			mrobotState.setToDefaultValues(mrobotState.getJointModelGroup(groupName), "home_table_left_torso");
			group.setStartState(mrobotState);
			target_pose1.position.x = target.position[0];
			target_pose1.position.y = target.position[1];
			target_pose1.position.z = target.position[2];
			group.setPoseTarget(target_pose1,"left_palm");
			group.setGoalPositionTolerance(target.dimension[0]);

			ROS_INFO("=========================================================");
			ROS_INFO("Pose goal planning with obstacle with KPIECE");
			ROS_INFO("=========================================================");
			group.setPlannerId("KPIECEkConfigDefault");
			success = group.plan(my_plan);
			ROS_INFO("Visualizing plan 10 (pose goal move around box) %s",
			success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(.5);
			if (success)
			{
				/* Print end-effector pose. Remember that this is in the model frame */
				vector<geometry_msgs::Pose> trajCartEE, trajCartElbow;
				for (int i=0; i<simpleTrajectory.points.size(); i++)
				{
					mrobotState.setJointGroupPositions(joint_model_group,simpleTrajectory.points[i].positions);
					end_effector_state = mrobotState.getGlobalLinkTransform("left_palm");
					elbow_state = mrobotState.getGlobalLinkTransform("la4");
					addValueTrajCart(end_effector_state,trajCartEE);
					addValueTrajCart(elbow_state,trajCartElbow);

				}
				double costEE = computeMotionDistance(trajCartEE);
				double costElbow = computeMotionDistance(trajCartElbow);

				ROS_INFO("cost EE= %f",costEE);
				ROS_INFO("cost Elbow= %f",costElbow);

				costEE_all.push_back(costEE);
				costElbow_all.push_back(costElbow);
			}
			else
			{
				costEE_all.push_back(-1.0);
				costElbow_all.push_back(-1.0);
			}

			// export results to file and clear old result
			logBatchSummary(countReplan);

			ROS_INFO("all planning time");
			for (int i=0; i<planningTime_all.size(); i++)
			{
				ROS_INFO("\t%f",planningTime_all[i]);
			}

			ROS_INFO("all cost EE");
			for (int i=0; i<costEE_all.size(); i++)
			{
				ROS_INFO("\t%f",costEE_all[i]);
			}

			ROS_INFO("all cost Elbow");
			for (int i=0; i<costElbow_all.size(); i++)
			{
				ROS_INFO("\t%f",costElbow_all[i]);
			}

			costEE_all.clear();
			costElbow_all.clear();
			planningTime_all.clear();


			obsSet.clear();
			planning_scene_interface.removeCollisionObjects(object_ids);
			yarp_msg::SharedData_new msg;
			msg.text = "finished";
			msg.content.push_back(1);
			pub_signal2Yarp.publish(msg);
			ROS_INFO("Waiting...");
		}
//		else
//		{
//			yarp_msg::SharedData_new msg;
//			msg.text = "replan";
//			msg.content.push_back(1);
//			pub_signal2Yarp.publish(msg);
////			ROS_INFO("Waiting...");
//		}

	}
	// END_TUTORIAL

	ros::shutdown();
	return 0;
}

//void computeMotionDistance(string linkName, const trajectory_msgs::JointTrajectory& trajJoint,
//		const Eigen::Affine3d& linkState, const RobotState& mState, const robot_state::JointModelGroup* mJMG)
//{
//	linkState = mState.getGlobalLinkTransform(linkName);
//	vector<geometry_msgs::Pose> trajCart;
//	for (int i=0; i<trajJoint.points.size(); i++)
//	{
//		mState.setJointGroupPositions(mJMG,trajJoint.points[i].positions);
//	}
//}

void addValueTrajCart(const Eigen::Affine3d& mState, vector<geometry_msgs::Pose>& trajCart)
{
	Eigen::Vector3d jointPose;
	jointPose = mState.translation();
//	trajCart.resize(3);
	geometry_msgs::Pose pointCart;
	pointCart.position.x = jointPose.x();
	pointCart.position.y = jointPose.y();
	pointCart.position.z = jointPose.z();

	trajCart.push_back(pointCart);
}

double distancePoint2Point(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
	double dist = sqrt(pow(p1.position.x-p2.position.x,2.0)+
			pow(p1.position.y-p2.position.y,2.0)+
			pow(p1.position.z-p2.position.z,2.0));


	return dist;
}

double computeMotionDistance(vector<geometry_msgs::Pose>& trajCart)
{
	double cost=0.0;
	for (int i=0; i<trajCart.size()-1; i++)
		cost += distancePoint2Point(trajCart[i],trajCart[i+1]);

	return cost;
}

void initBatchSummary()
{
    ofstream logfile1(fileName,ofstream::out | ofstream::trunc);
    if (logfile1.is_open())
    {
        logfile1<<"count"<<"\t";
        logfile1<<"numberObstacles"<<"\t";
        logfile1<<"obsSet"<<"\t";

        logfile1<<"RRTstar-Time"<<"\t";
        logfile1<<"RRTstar-CostEE"<<"\t";
        logfile1<<"RRTstar-CostEB"<<"\t";

        logfile1<<"RRTConnect-Time"<<"\t";
        logfile1<<"RRTConnect-CostEE"<<"\t";
        logfile1<<"RRTConnect-CostEB"<<"\t";

        logfile1<<"PRMstar-Time"<<"\t";
        logfile1<<"PRMstar-CostEE"<<"\t";
        logfile1<<"PRMstar-CostEB"<<"\t";

        logfile1<<"SBL-Time"<<"\t";
        logfile1<<"SBL-CostEE"<<"\t";
        logfile1<<"SBL-CostEB"<<"\t";

        logfile1<<"EST-Time"<<"\t";
        logfile1<<"EST-CostEE"<<"\t";
        logfile1<<"EST-CostEB"<<"\t";

        logfile1<<"LBKPIECE-Time"<<"\t";
        logfile1<<"LBKPIECE-CostEE"<<"\t";
        logfile1<<"LBKPIECE-CostEB"<<"\t";

        logfile1<<"BKPIECE-Time"<<"\t";
        logfile1<<"BKPIECE-CostEE"<<"\t";
        logfile1<<"BKPIECE-CostEB"<<"\t";

        logfile1<<"KPIECE-Time"<<"\t";
        logfile1<<"KPIECE-CostEE"<<"\t";
        logfile1<<"KPIECE-CostEB"<<"\n";


    }
    else
    {
    	ROS_ERROR("Cannot open log file");

    }
    logfile1.close();
}

void logBatchSummary(const int& count)
{
    ofstream logfile1(fileName,ofstream::out | ofstream::app);

    if (logfile1.is_open())
    {
        logfile1<<count<<"\t";
        logfile1<<(int)numberObstacles<<"\t";
        logfile1<<obsSet.size()<<"\t";

        for (int i=0; i<planningTime_all.size(); i++)
        {
			logfile1<<planningTime_all[i]<<"\t";
			logfile1<<costEE_all[i]<<"\t";
			logfile1<<costElbow_all[i]<<"\t";
        }

        logfile1<<"\n";


    }
    else
    {
    	ROS_ERROR("Cannot open log file ");

    }
    logfile1.close();
}

