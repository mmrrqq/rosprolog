#ifndef __ROSPROLOG_KB_H__
#define __ROSPROLOG_KB_H__

#include <rclcpp/rclcpp.hpp>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <string>

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rosprolog/rosprolog_node/PrologPool.h>

namespace rosprolog_kb {
	/**
	 * A ROS node supposed to be used to access ROS 
	 * from within the KB.
	 **/
	rclcpp::Node* node();
	
	/**
	 * A pool of Prolog engines to issue queries
	 * in C++ code.
	 **/
	PrologPool& thread_pool();
	
	void term_to_color(const PlTerm &term, std_msgs::msg::ColorRGBA &value);
	
	void term_to_vector3(const PlTerm &term, geometry_msgs::msg::Vector3 &value);
	
	void term_to_point(const PlTerm &term, geometry_msgs::msg::Point &value);
	
	void term_to_quaternion(const PlTerm &term, geometry_msgs::msg::Quaternion &value);
	
	void term_to_pose_stamped(const PlTerm &term, geometry_msgs::msg::PoseStamped &value);
	
	void term_to_transform_stamped(const PlTerm &term, geometry_msgs::msg::TransformStamped &value);
};

#endif
