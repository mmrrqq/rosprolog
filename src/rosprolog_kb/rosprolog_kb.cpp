#include "rosprolog/rosprolog_kb/rosprolog_kb.h"
#include "private/rosprolog_kb_priv.h"

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <console_bridge/console.h>

/*********************************/
/********** KBNode ***************/
/*********************************/
    
rosprolog_kb::KBNode::KBNode(): rclcpp::Node("~"),
	thread_(&KBNode::run, this),
	thread_pool_(2)
{
}

rosprolog_kb::KBNode::~KBNode()
{
	rclcpp::shutdown();
	thread_.join();
}

rosprolog_kb::KBNode& rosprolog_kb::KBNode::get()
{
	if(!rclcpp::ok()) {
		int argc=0;
		rclcpp::init(argc, (char**)NULL);
	}
	static std::shared_ptr<rosprolog_kb::KBNode> the_node = std::make_shared<rosprolog_kb::KBNode>();
	return *the_node.get();
}

rclcpp::Node* rosprolog_kb::KBNode::node()
{
	return static_cast<rclcpp::Node*>(&KBNode::get());
}

PrologPool& rosprolog_kb::KBNode::thread_pool()
{
	return KBNode::get().thread_pool_;
}

void rosprolog_kb::KBNode::run()
{
	RCLCPP_DEBUG(rclcpp::get_logger("rosprolog_kbnode"),"rosprolog_kb thread started.");
	
	rclcpp::executors::MultiThreadedExecutor spinner(rclcpp::ExecutorOptions(), 4);
	spinner.spin();
	
	RCLCPP_DEBUG(rclcpp::get_logger("rosprolog_kbnode"),"rosprolog_kb thread stopped.");
}

rclcpp::Node* rosprolog_kb::node()
{
	return rosprolog_kb::KBNode::node();
}

PrologPool& rosprolog_kb::thread_pool()
{
	return rosprolog_kb::KBNode::thread_pool();
}

PREDICATE(ros_init, 0) {
	rosprolog_kb::KBNode::node();
	return TRUE;
}

/*********************************/
/********** Logging **************/
/*********************************/

PREDICATE(ros_info, 1)
{
	RCLCPP_INFO(rclcpp::get_logger("rosprolog_kb"),"%s", (char*)PL_A1);
	return TRUE;
}

PREDICATE(ros_warn, 1)
{
	RCLCPP_WARN(rclcpp::get_logger("rosprolog_kb"),"%s", (char*)PL_A1);
	return TRUE;
}

PREDICATE(ros_error, 1)
{
	RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"),"%s", (char*)PL_A1);
	return TRUE;
}

PREDICATE(ros_debug, 1)
{
	RCLCPP_DEBUG(rclcpp::get_logger("rosprolog_kb"),"%s", (char*)PL_A1);
	return TRUE;
}

// https://github.com/ros2/demos/blob/humble/logging_demo/src/logger_config_component.cpp
PREDICATE(ros_set_logger_level, 1)
{
	const std::string severity_string = (char*)PL_A1;
	int severity;
  	rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
    	severity_string.c_str(), rcl_get_default_allocator(), &severity);

  	if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
    	RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"), "Unknown severity '%s'", severity_string.c_str());
    	return FALSE;
  	}

  	if (RCUTILS_RET_OK != ret) {
    	RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"), "Error %d getting severity level from request: %s", ret,
      		rcutils_get_error_string().str);
    	rcl_reset_error();
    	return FALSE;
  	}

  	ret = rcutils_logging_set_logger_level("rosprolog_kb", severity);
  	if (ret != RCUTILS_RET_OK) {
    	RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"), "Error setting severity: %s", rcutils_get_error_string().str);
    	rcutils_reset_error();
		return FALSE;
  	}

	return TRUE;
}

/*********************************/
/******** ROS Packages ***********/
/*********************************/

PREDICATE(ros_package_path, 2)
{
	std::string path = ament_index_cpp::get_package_share_directory(std::string((char*)PL_A1));
	if(path.empty()) return FALSE;
	PL_A2 = path.c_str();
	return TRUE;
}

PREDICATE(ros_package_command, 2)
{
	//PL_A2 = ros::package::command(std::string((char*)PL_A1)).c_str();
	return TRUE;
}
