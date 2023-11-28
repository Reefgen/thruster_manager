#include <thruster_manager/thruster_manager_node.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using namespace thruster_manager;

using geometry_msgs::msg::WrenchStamped;

rcl_interfaces::msg::ParameterDescriptor description(const std::string &des)
{
  return rcl_interfaces::msg::ParameterDescriptor().set__description(des);
}

ThrusterManagerNode::ThrusterManagerNode(rclcpp::NodeOptions options)
  : Node("thruster_manager", options)
{

  const auto pub_js{declare_parameter("publish_joint_state", true,  description("If the command should be published as joint state"))};
  const auto pub_gz{declare_parameter("publish_gz_command", true, description("If the command should be published as Float64 (Gazebo thruster plugin)"))};
  const auto sub_stamped{declare_parameter("subscribe_stamped", false,
                                           description("If the node should expect WrenchStamped messages instead of Wrench"))};
  const auto control_frame{declare_parameter<std::string>("control_frame", "base_link")};

  // joint names are stored here anyway to sync joint names and indices
  const auto links{allocator.parseRobotDescription(this, control_frame)};
  std::transform(links.begin(), links.end(), std::back_inserter(cmd.name), [](const auto &link){return link.joint;});
  dofs = cmd.name.size();
  cmd.effort.resize(dofs);

  if(pub_js)
    cmd_js_pub = create_publisher<JointState>("cmd_thrust", 5);

  if(pub_gz)
  {
    for(const auto &name: cmd.name)
    {
      const auto topic{"cmd_" + name};
      cmd_gz_pub.push_back(create_publisher<Float64>(topic,5));
    }
  }

  if(sub_stamped)
  {
    wrench_sub = create_subscription<WrenchStamped>("wrench", 5, [&](const WrenchStamped::SharedPtr msg)
    {
      solve(msg->wrench);
    });
  }
  else
  {
    wrench_sub = create_subscription<Wrench>("wrench", 5, [&](const Wrench::SharedPtr msg)
    {
      solve(*msg);
    });
  }

  max_wrench_pub = this -> create_publisher<std_msgs::msg::Float64MultiArray>("max_wrench", 10);
  max_wrench_array.resize(6);
  std::fill(max_wrench_array.begin(),max_wrench_array.end(),0);

  min_wrench_pub = this -> create_publisher<std_msgs::msg::Float64MultiArray>("min_wrench", 10);
  min_wrench_array.resize(6);
  std::fill(min_wrench_array.begin(),min_wrench_array.end(),0);

  timer = this->create_wall_timer(50ms, std::bind(&ThrusterManagerNode::timer_callback, this));
}


void ThrusterManagerNode::solve(const Wrench &wrench)
{
  ThrusterManager::Vector6d F;
  F(0) = wrench.force.x;
  F(1) = wrench.force.y;
  F(2) = wrench.force.z;
  F(3) = wrench.torque.x;
  F(4) = wrench.torque.y;
  F(5) = wrench.torque.z;

  const auto thrusts{allocator.solveWrench(F)};

  if(cmd_js_pub)
  {
    std::copy(thrusts.data(), thrusts.data()+dofs, cmd.effort.begin());
    cmd.header.stamp = get_clock()->now();
    cmd_js_pub->publish(cmd);
  }

  if(!cmd_gz_pub.empty())
  {
    static Float64 cmd_gz;
    for(size_t i = 0; i < dofs; ++i)
    {
      cmd_gz.data = thrusts[i];
      cmd_gz_pub[i]->publish(cmd_gz);
    }
  }
}

void ThrusterManagerNode::timer_callback()
{
  ThrusterManager::Vector6d max_temp = allocator.maxWrench();
  ThrusterManager::Vector6d min_temp = allocator.minWrench();
  for(uint i=0; i<6; i++)
  {
    max_wrench_array[i] = max_temp[i];
    min_wrench_array[i] = min_temp[i];

  }
  std_msgs::msg::Float64MultiArray max_wrench_msg;
  std_msgs::msg::Float64MultiArray min_wrench_msg;
  max_wrench_msg.set__data(max_wrench_array);
  min_wrench_msg.set__data(min_wrench_array);
  max_wrench_pub->publish(max_wrench_msg);
  min_wrench_pub->publish(min_wrench_msg);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(thruster_manager::ThrusterManagerNode)
