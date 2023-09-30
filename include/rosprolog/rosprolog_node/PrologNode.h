#ifndef __ROSPROLOG_NODE_H__
#define __ROSPROLOG_NODE_H__

//STD
#include <string>
#include <memory>
#include <iostream>

// rclcpp
#include <rclcpp/rclcpp.hpp>
// boost
#include <boost/shared_ptr.hpp>
// SWI Prolog
#include <SWI-Prolog.h>
// rosprolog
#include <rosprolog/rosprolog_node/PrologPool.h>
#include <json_prolog_msgs/srv/prolog_query.hpp>
#include <json_prolog_msgs/srv/prolog_finish.hpp>
#include <json_prolog_msgs/srv/prolog_next_solution.hpp>

/**
 * ROS service interface to rosprolog
 *
 * @author Daniel Beßler
 */
class PrologNode : public rclcpp::Node {
public:
	PrologNode(const char* ns);
	
	bool query(const std::shared_ptr<json_prolog_msgs::srv::PrologQuery_Request> req,
	           std::shared_ptr<json_prolog_msgs::srv::PrologQuery_Response> res);
	
	void finalize();
	bool finish(const std::shared_ptr<json_prolog_msgs::srv::PrologFinish_Request> req,
		        std::shared_ptr<json_prolog_msgs::srv::PrologFinish_Response> res);
	
	bool next_solution(const std::shared_ptr<json_prolog_msgs::srv::PrologNextSolution_Request> req,
		               std::shared_ptr<json_prolog_msgs::srv::PrologNextSolution_Response> res);
	
	bool is_initialized() { return is_initialized_; }
	
private:
	bool is_initialized_ = false;
	PrologPool thread_pool_;
	std::map< std::string, boost::shared_ptr<PrologEngine> > claimed_engines_;
	
	rclcpp::Service<json_prolog_msgs::srv::PrologQuery>::SharedPtr prolog_query_;
	rclcpp::Service<json_prolog_msgs::srv::PrologNextSolution>::SharedPtr prolog_next_solution_;
	rclcpp::Service<json_prolog_msgs::srv::PrologFinish>::SharedPtr prolog_finish_;

	bool exists(const std::string &id);
	
	void finalize(const std::string &id);
	
	bool has_more_solutions(const std::string &id);
	
	static int call1(const std::string &predicate, const std::string &arg1);
	
	static int ensure_loaded(const char *ros_pkg);
	
	static int num_pl_threads(rclcpp::Node *node);
};

#endif //__ROSPROLOG_NODE_H__
