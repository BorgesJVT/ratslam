/*
 * openRatSLAM
 *
 * main_lv - ROS interface bindings for the local view cells
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <memory>
#include <functional>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <topological_msgs/msg/view_template.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include "ratslam/local_view_match.h"

#if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
#endif


using namespace ratslam;

class LocalViewNode : public rclcpp::Node
{
public:
  LocalViewNode()
    : Node("ratslam_lv")
  {
    // Declare and get parameters
    this->declare_parameter("topic_root", "");
    this->declare_parameter("image_crop_y_max", -1);
    this->declare_parameter("template_x_size", 1);
    this->declare_parameter("template_y_size", 1);
    this->declare_parameter("vt_shift_match", 25);
    this->declare_parameter("vt_step_match", 5);
    this->declare_parameter("vt_match_threshold", 0.03);
    this->declare_parameter("vt_min_patch_normalisation_std", 0.0);
    this->declare_parameter("vt_patch_normalise", 0);
    this->declare_parameter("vt_normalisation", 0.0);
    this->declare_parameter("vt_panoramic", 0);
    this->declare_parameter("image_crop_x_min", 0);
    this->declare_parameter("image_crop_x_max", -1);
    this->declare_parameter("image_crop_y_min", 0);
    this->declare_parameter("vt_active_decay", 1.0);
    this->declare_parameter("enable", true);
    this->declare_parameter("vt_window_width", 416);
    this->declare_parameter("vt_window_height", 480);
    this->declare_parameter("media_path", "");
    this->declare_parameter("image_file", "");
    
    std::string topic_root = this->get_parameter("topic_root").as_string();
    
    // Log all parameters for debugging
    RCLCPP_INFO(this->get_logger(), "LocalView Parameters:");
    RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
    RCLCPP_INFO(this->get_logger(), "  template_x_size: %ld", this->get_parameter("template_x_size").as_int());
    RCLCPP_INFO(this->get_logger(), "  template_y_size: %ld", this->get_parameter("template_y_size").as_int());
    RCLCPP_INFO(this->get_logger(), "  image_crop_x_min: %ld", this->get_parameter("image_crop_x_min").as_int());
    RCLCPP_INFO(this->get_logger(), "  image_crop_x_max: %ld", this->get_parameter("image_crop_x_max").as_int());
    RCLCPP_INFO(this->get_logger(), "  image_crop_y_min: %ld", this->get_parameter("image_crop_y_min").as_int());
    RCLCPP_INFO(this->get_logger(), "  image_crop_y_max: %ld", this->get_parameter("image_crop_y_max").as_int());
    RCLCPP_INFO(this->get_logger(), "  vt_shift_match: %ld", this->get_parameter("vt_shift_match").as_int());
    RCLCPP_INFO(this->get_logger(), "  vt_step_match: %ld", this->get_parameter("vt_step_match").as_int());
    RCLCPP_INFO(this->get_logger(), "  vt_match_threshold: %f", this->get_parameter("vt_match_threshold").as_double());
    RCLCPP_INFO(this->get_logger(), "  vt_min_patch_normalisation_std: %f", this->get_parameter("vt_min_patch_normalisation_std").as_double());
    RCLCPP_INFO(this->get_logger(), "  vt_patch_normalise: %ld", this->get_parameter("vt_patch_normalise").as_int());
    RCLCPP_INFO(this->get_logger(), "  vt_normalisation: %f", this->get_parameter("vt_normalisation").as_double());
    RCLCPP_INFO(this->get_logger(), "  vt_panoramic: %ld", this->get_parameter("vt_panoramic").as_int());
    
    // Create LocalViewMatch with parameters
    lv = new ratslam::LocalViewMatch(
      this->get_parameter("vt_min_patch_normalisation_std").as_double(),
      this->get_parameter("vt_patch_normalise").as_int(),
      this->get_parameter("vt_normalisation").as_double(),
      this->get_parameter("vt_shift_match").as_int(),
      this->get_parameter("vt_step_match").as_int(),
      this->get_parameter("vt_panoramic").as_int(),
      this->get_parameter("vt_match_threshold").as_double(),
      this->get_parameter("template_x_size").as_int(),
      this->get_parameter("template_y_size").as_int(),
      this->get_parameter("image_crop_x_min").as_int(),
      this->get_parameter("image_crop_x_max").as_int(),
      this->get_parameter("image_crop_y_min").as_int(),
      this->get_parameter("image_crop_y_max").as_int()
    );
    
    pub_vt = this->create_publisher<topological_msgs::msg::ViewTemplate>(
      topic_root + "/LocalView/Template", 10);
    
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10,
      std::bind(&LocalViewNode::image_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "LocalView node initialized");
    
#ifdef HAVE_IRRLICHT
    use_graphics = this->get_parameter("enable").as_bool();
    if (use_graphics)
    {
      lvs = new ratslam::LocalViewScene(
        this->get_parameter("vt_window_width").as_int(),
        this->get_parameter("vt_window_height").as_int(),
        lv
      );
    }
#endif
  }
  
  ~LocalViewNode()
  {
    if (lv != nullptr)
      delete lv;
#ifdef HAVE_IRRLICHT
    if (lvs != nullptr)
      delete lvs;
#endif
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image)
  {
    RCLCPP_DEBUG(this->get_logger(), "LV:image_callback seq=%d", image->header.stamp.sec);

    // Decode compressed image
    cv::Mat decoded_image;
    try
    {
      decoded_image = cv::imdecode(cv::Mat(image->data), cv::IMREAD_COLOR);
      if (decoded_image.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
        return;
      }
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
      return;
    }

    topological_msgs::msg::ViewTemplate vt_output;

    lv->on_image(decoded_image.data, false, decoded_image.cols, decoded_image.rows);

    vt_output.header.stamp = this->now();
    vt_output.current_id = lv->get_current_vt();
    vt_output.relative_rad = lv->get_relative_rad();

    pub_vt->publish(vt_output);

#ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
      lvs->draw_all();
    }
#endif
  }

  ratslam::LocalViewMatch* lv = nullptr;
  rclcpp::Publisher<topological_msgs::msg::ViewTemplate>::SharedPtr pub_vt;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
#ifdef HAVE_IRRLICHT
  ratslam::LocalViewScene *lvs = nullptr;
  bool use_graphics;
#endif
};

ratslam::LocalViewMatch * lv = NULL;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath" << std::endl;
  std::cout << "RatSLAM algorithm by Michael Milford and Gordon Wyeth" << std::endl;
  std::cout << "Distributed under the GNU GPL v3, see the included license file." << std::endl;

  auto node = std::make_shared<LocalViewNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
