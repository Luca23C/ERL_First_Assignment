// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 250ms)
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "/map";
    wp.header.stamp = now();
    wp.pose.position.x = -7.0;
    wp.pose.position.y = -1.5;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    waypoints_["w0"] = wp;

    wp.pose.position.x = -3.0;
    wp.pose.position.y = -8.0;
    waypoints_["w1"] = wp;

    wp.pose.position.x = 6.0;
    wp.pose.position.y = 2.0;
    waypoints_["w2"] = wp;

    wp.pose.position.x = -7.0;
    wp.pose.position.y = -5.0;
    waypoints_["w3"] = wp;

    wp.pose.position.x = -2.0;
    wp.pose.position.y = -0.4;
    waypoints_["center"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
    progress_ = 0.0;
  }

private:
  void do_work()
  {
  
    send_feedback(progress_, "Move running");
    if (progress_ < 1.0) {
      progress_ += 0.02;
      send_feedback(progress_, "Move running");
    } else {
      finish(true, 1.0, "Move completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  
 
    
  
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
