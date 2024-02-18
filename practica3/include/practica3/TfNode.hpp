// Copyright Rui Bartolome Segura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef PRACTICA3_TFNODE_HPP_
#define PRACTICA3_TFNODE_HPP_

#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/twist.hpp"


#include "rclcpp/rclcpp.hpp"

namespace practica3
{

class TfNode : public rclcpp::Node
{
public:
  TfNode();

private:
  void control_cycle();

  tf2::Transform get_tf();
  std::tuple<double, double, double, double> calculate_param();

  static const int FORWARD = 0;
  static const int TURN = 1;
  static const int STOP = 2;
  int state_;

  void go_state(int new_state);
  bool check_forward_2_turn();
  bool check_turn_2_stop();

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_STOP = 0.0f;
  static constexpr float SPEED_ANGULAR = 0.3f;

  double roll, pitch, yaw;
  static constexpr double pi = 3.14;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::Stamped<tf2::Transform> odom2bf0;
  tf2::Stamped<tf2::Transform> odom2bf1;
  std::string error;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  //  namespace PRACTICA3

#endif  // PRACTICA3_TfNode_HPP_
