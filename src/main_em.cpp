#include "ratslam/utils.h"
#include <boost/property_tree/ini_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <topological_msgs/msg/topological_action.hpp>
#include <topological_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ratslam/experience_map.h"

//#if HAVE_IRRLICHT
#include "graphics/experience_map_scene.h"
ratslam::ExperienceMapScene *ems = nullptr;
bool use_graphics;
//#endif

using namespace ratslam;

class RatSLAMExperienceMap : public rclcpp::Node {
public:
    RatSLAMExperienceMap() : Node("ratslam_experience_map") {
        RCLCPP_INFO(this->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
        RCLCPP_INFO(this->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
        RCLCPP_INFO(this->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

        this->declare_parameter<std::string>("config_file", "");
        std::string config_file;
        this->get_parameter("config_file", config_file);

        std::string topic_root;
        boost::property_tree::ptree settings, general_settings, ratslam_settings;
        read_ini(config_file, settings);
        get_setting_child(general_settings, settings, "general", true);
        get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
        get_setting_child(ratslam_settings, settings, "ratslam", true);

        em_ = std::make_unique<ratslam::ExperienceMap>(ratslam_settings);

        pub_em_ = this->create_publisher<topological_msgs::msg::TopologicalMap>(topic_root + "/ExperienceMap/Map", 10);
        pub_em_markers_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_root + "/ExperienceMap/MapMarker", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_root + "/ExperienceMap/RobotPose", 10);
        pub_goal_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_root + "/ExperienceMap/PathToGoal", 10);

        sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_root + "/odom", 10, std::bind(&RatSLAMExperienceMap::odo_callback, this, std::placeholders::_1));
        sub_action_ = this->create_subscription<topological_msgs::msg::TopologicalAction>(
            topic_root + "/PoseCell/TopologicalAction", 10, std::bind(&RatSLAMExperienceMap::action_callback, this, std::placeholders::_1));
        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_root + "/ExperienceMap/SetGoalPose", 10, std::bind(&RatSLAMExperienceMap::set_goal_pose_callback, this, std::placeholders::_1));

        //#ifdef HAVE_IRRLICHT
        boost::property_tree::ptree draw_settings;
        get_setting_child(draw_settings, settings, "draw", true);
        get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
        if (use_graphics) {
            ems = new ratslam::ExperienceMapScene(draw_settings, em_.get());
        }
        //#endif
    }

private:
    void odo_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "EM:odo_callback seq=%d", msg->header.frame_id);

        static rclcpp::Time prev_time(0, 0, RCL_ROS_TIME);
        if (prev_time.nanoseconds() > 0) {
            rclcpp::Time odo_time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
            double time_diff = (odo_time - prev_time).seconds();
            em_->on_odo(msg->twist.twist.linear.x, msg->twist.twist.angular.z, time_diff);
        }
        prev_time = msg->header.stamp;
    }

    void action_callback(const topological_msgs::msg::TopologicalAction::SharedPtr msg) {
  
      switch (msg->action) {
          case topological_msgs::msg::TopologicalAction::CREATE_NODE:
              em_->on_create_experience(msg->dest_id);
              em_->on_set_experience(msg->dest_id, 0);
              break;
          case topological_msgs::msg::TopologicalAction::CREATE_EDGE:
              em_->on_create_link(msg->src_id, msg->dest_id, msg->relative_rad);
              em_->on_set_experience(msg->dest_id, msg->relative_rad);
              break;
          case topological_msgs::msg::TopologicalAction::SET_NODE:
              em_->on_set_experience(msg->dest_id, msg->relative_rad);
              break;
      }
  
      em_->iterate();
  
      // Publicação da posição do robô
      geometry_msgs::msg::PoseStamped pose_output;
      pose_output.header.stamp = this->get_clock()->now();
      pose_output.pose.position.x = em_->get_experience(em_->get_current_id())->x_m;
      pose_output.pose.position.y = em_->get_experience(em_->get_current_id())->y_m;
      pose_output.pose.orientation.z = sin(em_->get_experience(em_->get_current_id())->th_rad / 2.0);
      pose_output.pose.orientation.w = cos(em_->get_experience(em_->get_current_id())->th_rad / 2.0);
  
      pub_pose_->publish(pose_output);
  
      // Publicação do mapa topológico em intervalos regulares
      static rclcpp::Time prev_pub_time(0, 0, RCL_ROS_TIME);
      rclcpp::Time msg_time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
      if (msg_time - prev_pub_time > rclcpp::Duration(std::chrono::seconds(30))) {

          prev_pub_time = msg->header.stamp;
  
          topological_msgs::msg::TopologicalMap em_map;
          em_map.header.stamp = this->get_clock()->now();
          em_map.node_count = em_->get_num_experiences();
          em_map.node.resize(em_->get_num_experiences());
          
          for (int i = 0; i < em_->get_num_experiences(); i++) {
              em_map.node[i].id = em_->get_experience(i)->id;
              em_map.node[i].pose.position.x = em_->get_experience(i)->x_m;
              em_map.node[i].pose.position.y = em_->get_experience(i)->y_m;
              em_map.node[i].pose.orientation.z = sin(em_->get_experience(i)->th_rad / 2.0);
              em_map.node[i].pose.orientation.w = cos(em_->get_experience(i)->th_rad / 2.0);
          }
  
          em_map.edge_count = em_->get_num_links();
          em_map.edge.resize(em_->get_num_links());
          for (int i = 0; i < em_->get_num_links(); i++) {
              em_map.edge[i].source_id = em_->get_link(i)->exp_from_id;
              em_map.edge[i].destination_id = em_->get_link(i)->exp_to_id;
              em_map.edge[i].transform.translation.x = em_->get_link(i)->d * cos(em_->get_link(i)->heading_rad);
              em_map.edge[i].transform.translation.y = em_->get_link(i)->d * sin(em_->get_link(i)->heading_rad);
              em_map.edge[i].transform.rotation.z = sin(em_->get_link(i)->facing_rad / 2.0);
              em_map.edge[i].transform.rotation.w = cos(em_->get_link(i)->facing_rad / 2.0);
          }
  
          pub_em_->publish(em_map);
      }
  
      // Publicação dos marcadores de visualização
      visualization_msgs::msg::Marker em_marker;
      em_marker.header.stamp = this->get_clock()->now();
      em_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      em_marker.points.resize(em_->get_num_links() * 2);
      em_marker.action = visualization_msgs::msg::Marker::ADD;
      em_marker.scale.x = 0.01;
      em_marker.color.a = 1;
      em_marker.ns = "em";
  
      for (int i = 0; i < em_->get_num_links(); i++) {
          em_marker.points[i * 2].x = em_->get_experience(em_->get_link(i)->exp_from_id)->x_m;
          em_marker.points[i * 2].y = em_->get_experience(em_->get_link(i)->exp_from_id)->y_m;
          em_marker.points[i * 2 + 1].x = em_->get_experience(em_->get_link(i)->exp_to_id)->x_m;
          em_marker.points[i * 2 + 1].y = em_->get_experience(em_->get_link(i)->exp_to_id)->y_m;
      }
  
      pub_em_markers_->publish(em_marker);
  
      //#ifdef HAVE_IRRLICHT
      if (use_graphics) {
          ems->update_scene();
          ems->draw_all();
      }
      //#endif
    }
  
    void set_goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        em_->add_goal(msg->pose.position.x, msg->pose.position.y);
    }

    std::unique_ptr<ratslam::ExperienceMap> em_;
    rclcpp::Publisher<topological_msgs::msg::TopologicalMap>::SharedPtr pub_em_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_em_markers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_goal_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<topological_msgs::msg::TopologicalAction>::SharedPtr sub_action_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RatSLAMExperienceMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
