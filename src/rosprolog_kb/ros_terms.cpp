#include <rosprolog/rosprolog_kb/rosprolog_kb.h>

#include <rclcpp/rclcpp.hpp>

namespace rosprolog_kb {
    void term_to_color(const PlTerm &rosterm, std_msgs::msg::ColorRGBA &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.r = (double)e;
        list.next(e); value.g = (double)e;
        list.next(e); value.b = (double)e;
        list.next(e); value.a = (double)e;
    }
    
    void term_to_vector3(const PlTerm &rosterm, geometry_msgs::msg::Vector3 &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.x = (double)e;
        list.next(e); value.y = (double)e;
        list.next(e); value.z = (double)e;
    }
    
    void term_to_point(const PlTerm &rosterm, geometry_msgs::msg::Point &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.x = (double)e;
        list.next(e); value.y = (double)e;
        list.next(e); value.z = (double)e;
    }
    
    void term_to_quaternion(const PlTerm &rosterm, geometry_msgs::msg::Quaternion &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.x = (double)e;
        list.next(e); value.y = (double)e;
        list.next(e); value.z = (double)e;
        list.next(e); value.w = (double)e;
    }
    
    void term_to_transform_stamped(const PlTerm &rosterm, geometry_msgs::msg::TransformStamped &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.header.frame_id = (char*)e;
        list.next(e); value.child_frame_id = (char*)e;
        list.next(e); term_to_vector3(e, value.transform.translation);
        list.next(e); term_to_quaternion(e, value.transform.rotation);
    }
    
    void term_to_pose_stamped(const PlTerm &rosterm, geometry_msgs::msg::PoseStamped &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.header.frame_id = (char*)e;
        list.next(e); // unused
        list.next(e); term_to_point(e, value.pose.position);
        list.next(e); term_to_quaternion(e, value.pose.orientation);
    }
};
