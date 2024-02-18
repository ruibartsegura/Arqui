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

#include "rclcpp/rclcpp.hpp"

#include "practica3/TfNode.hpp"

namespace practica3 {

using namespace std::chrono_literals;

TfNode::TfNode()
: Node("TfNode"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf2::Transform odom2bf = get_tf();
  odom2bf0.setData(odom2bf);
  

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&TfNode::control_cycle, this));
}

tf2::Transform TfNode::get_tf()
{
  tf2::Stamped<tf2::Transform> odom2bf;
  std::string error;
  bool done = false;

  do {
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
      auto odom2bf_msg = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(odom2bf_msg, odom2bf);
      done = true;
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Error in TF odom -> base_footprint [<< " << error << "]");
    }
  } while(!done);

  return odom2bf;
}

void TfNode::control_cycle()
{
  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_turn()) {
        RCLCPP_INFO(get_logger(), "1m complete");
        go_state(TURN);
      }
      break;
    case TURN:
      out_vel.angular.z = SPEED_ANGULAR;

      if (check_turn_2_stop()) {
        RCLCPP_INFO(get_logger(), "pi rad turn complete");
        go_state(STOP);
      }
      break;
    case STOP:
      out_vel.linear.x = SPEED_STOP;
      RCLCPP_INFO(get_logger(), "Stop");
      break;
  }

  vel_pub_->publish(out_vel);
}

std::tuple<double, double, double, double> 
TfNode::calculate_param()
{
  tf2::Stamped<tf2::Transform> bf0_2bf1;
  double dist_, roll_, pitch_, yaw_;
  
  try {
    bf0_2bf1.setData(odom2bf0.inverseTimes(get_tf()));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_STREAM(get_logger(), "Error calculating bf0_2bf1 transform: " << ex.what());
    // Return default values
    return std::make_tuple(0.0, 0.0, 0.0, 0.0);
  }

  double x = bf0_2bf1.getOrigin().x();
  double y = bf0_2bf1.getOrigin().y();

  dist_ = sqrt(x * x + y * y);

  bf0_2bf1.getBasis().getRPY(roll_, pitch_, yaw_);

  return std::make_tuple(dist_, roll_, pitch_, yaw_);
}

void TfNode::go_state(int new_state)
{
  state_ = new_state;
}

bool TfNode::check_forward_2_turn()
{
  auto [dist, roll, pitch, yaw]  = calculate_param();
  if (dist >= 1.0) {
    return true;
  }
  
  return false;
}

bool TfNode::check_turn_2_stop()
{
  auto [dist, roll, pitch, yaw] = calculate_param();
  // Verificar si el robot ha girado pi radianes
  if (fabs(yaw) >= 3.1) {  // 3.1 radianes para que salte siempre y no haya error cuando empiece a disminuir
    return true;
  }
  return false;
}

}  // namespace practica3

