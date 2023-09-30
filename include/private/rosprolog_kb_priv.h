#ifndef __ROSPROLOG_KB__PRIV_H__
#define __ROSPROLOG_KB__PRIV_H__

// STD
#include <thread>
// ROS
#include <rclcpp/rclcpp.hpp>
// rosprolog
#include <rosprolog/rosprolog_node/PrologPool.h>

#include <rclcpp/rclcpp.hpp>

namespace rosprolog_kb {
	/**
	 * A ROS node that is supposed to be used from
	 * Prolog predicates to interact with ROS.
	 **/
	class KBNode : public rclcpp::Node {
	public:
		/**
		 * A pool of Prolog engines that can be used
		 * in C++ code to call queries without
		 * going via the *rosprolog_node*.
		 */
		static PrologPool& thread_pool();
		static rclcpp::Node* node();
		static KBNode& get();

		KBNode();
		~KBNode();
	private:
		std::thread thread_;
		PrologPool thread_pool_;
		void run();
	};
};

#endif
