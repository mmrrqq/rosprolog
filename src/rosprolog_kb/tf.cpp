#include <rosprolog/rosprolog_kb/rosprolog_kb.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define ROSPROLOG_TF_CACHE_TIME 10.0

std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf2_listener = nullptr;
tf2_ros::Buffer* tf_buffer() {
  if(tf2_buffer==nullptr) {
    rclcpp::Clock::SharedPtr system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_buffer =
      std::make_unique<tf2_ros::Buffer>(system_clock);
    tf2_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
    //listener = new tf::TransformListener(rclcpp::Duration(ROSPROLOG_TF_CACHE_TIME, 0));
  }
  return tf2_buffer.get();
}

void tf_pl_position(PlTermv &out, const geometry_msgs::msg::Point &v) {
  PlTail l(out[0]);
  l.append(v.x);
  l.append(v.y);
  l.append(v.z);
  l.close();
}

void tf_pl_to_point(const PlTerm &arg, geometry_msgs::msg::PointStamped &p) {
  PlTail list(arg); PlTerm value;
  list.next(value); p.point.x = value;
  list.next(value); p.point.y = value;
  list.next(value); p.point.z = value;
}

void tf_point_to_pl(const geometry_msgs::msg::PointStamped &p, const PlTerm &term) {
  PlTail l(term);
  l.append(p.point.x);
  l.append(p.point.y);
  l.append(p.point.z);
  l.close();
}

void tf_pl_quaternion(PlTermv &out, const geometry_msgs::msg::Quaternion &q) {
  PlTail l(out[1]);
  l.append(q.x);
  l.append(q.y);
  l.append(q.z);
  l.append(q.w);
  l.close();
}

void tf_pl_to_quaternion(const PlTerm &arg, geometry_msgs::msg::QuaternionStamped &p) {
  PlTail list(arg); PlTerm value;
  list.next(value); p.quaternion.x = value;
  list.next(value); p.quaternion.y = value;
  list.next(value); p.quaternion.z = value;
  list.next(value); p.quaternion.w = value;
}

void tf_pl_quaternion(PlTermv &out, const tf2::Quaternion &q) {
  PlTail l(out[1]);
  l.append(q.x());
  l.append(q.y());
  l.append(q.z());
  l.append(q.w());
  l.close();
}

void tf_quaternion_to_pl(const geometry_msgs::msg::QuaternionStamped &p, const PlTerm &term) {
  PlTail l(term);
  l.append(p.quaternion.x);
  l.append(p.quaternion.y);
  l.append(p.quaternion.z);
  l.append(p.quaternion.w);
  l.close();
}

void tf_pl_to_pose(const PlTerm &arg, geometry_msgs::msg::PoseStamped &p) {
  PlTail pos_list(arg[1]), rot_list(arg[2]); PlTerm value;
  pos_list.next(value); p.pose.position.x    = value;
  pos_list.next(value); p.pose.position.y    = value;
  pos_list.next(value); p.pose.position.z    = value;
  rot_list.next(value); p.pose.orientation.x = value;
  rot_list.next(value); p.pose.orientation.y = value;
  rot_list.next(value); p.pose.orientation.z = value;
  rot_list.next(value); p.pose.orientation.w = value;
}

void tf_pl_vector(PlTermv &out, const geometry_msgs::msg::Vector3 &v) {
  PlTail l(out[0]);
  l.append(v.x);
  l.append(v.y);
  l.append(v.z);
  l.close();
}

PREDICATE(tf_listener_start, 0) {
  tf_buffer();
  return TRUE;
}

// tf_lookup_transform(TargetFrame, SourceFrame, Transform)
PREDICATE(tf_lookup_transform, 3) {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer()->lookupTransform((char*)PL_A1, (char*)PL_A2, rclcpp::Time(0));
  }
  catch (const tf2::TransformException& ex){
    RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"),"%s",ex.what());
    return FALSE;
  }
  // create term `pose([X,Y,Z], [QX,QY,QZ,QW])`
  PlTermv pose(2);
  tf_pl_vector(pose, transform.transform.translation);
  tf_pl_quaternion(pose, transform.transform.rotation);
  PL_A3 = PlCompound("pose", pose);
  return TRUE;
}

// tf_transform_point(SourceFrame, TargetFrame, PointIn, PointOut)
PREDICATE(tf_transform_point, 4) {
  const char* source_frame = (char*)PL_A1;
  const char* target_frame = (char*)PL_A2;
  // read input point
  geometry_msgs::msg::PointStamped in,out;
  tf_pl_to_point(PL_A3, in);
  in.header.stamp = rclcpp::Time();
  in.header.frame_id = source_frame;
  // transform into target_frame
  try {
    tf_buffer()->transform(in, out, target_frame);
  }
  catch (const tf2::TransformException& ex){
    RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"),"%s",ex.what());
    return FALSE;
  }
  // write point output
  tf_point_to_pl(out, PL_A4);
  return TRUE;
}

// tf_transform_quaternion(SourceFrame, TargetFrame, QuaternionIn, QuaternionOut)
PREDICATE(tf_transform_quaternion, 4) {
  const char* source_frame = (char*)PL_A1;
  const char* target_frame = (char*)PL_A2;
  // read input point
  geometry_msgs::msg::QuaternionStamped in,out;
  tf_pl_to_quaternion(PL_A3, in);
  in.header.stamp = rclcpp::Time();
  in.header.frame_id = source_frame;
  // transform into target_frame
  try {
    tf_buffer()->transform(in, out, target_frame);
  }
  catch (const tf2::TransformException& ex){
    RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"),"%s",ex.what());
    return FALSE;
  }
  // write point output
  tf_quaternion_to_pl(out, PL_A4);
  return TRUE;
}

// tf_transform_pose(SourceFrame, TargetFrame, PoseIn, PoseOut)
PREDICATE(tf_transform_pose, 4) {
  const char* source_frame = (char*)PL_A1;
  const char* target_frame = (char*)PL_A2;
  // read input point
  geometry_msgs::msg::PoseStamped in,out;
  tf_pl_to_pose(PL_A3, in);
  in.header.stamp = rclcpp::Time();
  in.header.frame_id = source_frame;
  // transform into target_frame
  try {
    tf_buffer()->transform(in, out, target_frame);
  }
  catch (const tf2::TransformException& ex){
    RCLCPP_ERROR(rclcpp::get_logger("rosprolog_kb"),"%s",ex.what());
    return FALSE;
  }
  // write point output
  PlTermv pose(2);
  tf_pl_position(pose, out.pose.position);
  tf_pl_quaternion(pose, out.pose.orientation);
  PL_A4 = PlCompound("pose", pose);
  return TRUE;
}
