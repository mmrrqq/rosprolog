#ifndef __JSON_ROS_PRIV_H__
#define __JSON_ROS_PRIV_H__

//STD
#include <string>
#include <list>
#include <map>
// ROS
#include <rclcpp/rclcpp.hpp>
// rosprolog
#include <rosprolog/srv/json_wrapper.hpp>
#include <rosprolog/msg/message_json.hpp>

namespace json_ros {
	/**
	 * Receives jsonified ROS messages and calls a Prolog
	 * predicate with the JSON string as an argument.
	 * 
	 * @author Daniel Beßler
	 */
	class Subscriber {
	public:
		Subscriber(const std::string &id, const std::string &topic, const term_t &callback);
		
		const std::string& id() { return id_; }
		const std::string& topic() { return topic_; }
		const term_t& callback() { return callback_; }
		
		void handle_message(const std::string &message_json);
		
	private:
		std::string id_;
		std::string topic_;
		std::string callback_str_;
		term_t callback_;
	};

	/**
	 * Send JSON-encoded ROS commands for messages, services,
	 * and actions to the *json_ros* node.
	 * 
	 * @author Daniel Beßler
	 */
	class Wrapper {
	public:
		static Wrapper& get();
		
		rosprolog::srv::JSONWrapper_Response call(const std::string &mode, const std::string &json_data);
		
		rosprolog::srv::JSONWrapper_Response subscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data,
			const term_t &callback);
		
		rosprolog::srv::JSONWrapper_Response unsubscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data);
		
		void handle_message(const rosprolog::msg::MessageJSON::SharedPtr message);
		
	private:
		rclcpp::Node* node_ = nullptr;
		std::map< std::string, std::list<json_ros::Subscriber> > subscribers_;
		rclcpp::Client<rosprolog::srv::JSONWrapper>::SharedPtr service_;
		rclcpp::Subscription<rosprolog::msg::MessageJSON>::SharedPtr subscriber_;
		
		Wrapper(rclcpp::Node *nh);
		~Wrapper();
		
		Wrapper(Wrapper const&); // Don't Implement
		void operator=(Wrapper const&);     // Don't implement
	};
};

#endif //__JSON_ROS_PRIV_H__
