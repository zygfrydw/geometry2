/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <thread>

#include "node_wrapper.hpp"

class CustomNode : public rclcpp::Node
{
public:
  CustomNode() : rclcpp::Node("tf2_ros_test_transform_listener_node")
  {
  }

  void init_tf_listener()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer, shared_from_this(), false);
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

TEST(tf2_test_transform_listener, transform_listener_rclcpp_node)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, node, false);
}

TEST(tf2_test_transform_listener, transform_listener_custom_rclcpp_node)
{
  auto node = std::make_shared<NodeWrapper>("tf2_ros_message_filter");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, node, false);
}

TEST(tf2_test_transform_listener, transform_listener_as_member)
{
  auto custom_node = std::make_shared<CustomNode>();
  custom_node->init_tf_listener();
}

TEST(tf2_test_transform_listener, transform_listener_should_destroy_correctly)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  std::promise<void> promise;

  std::stringstream sstream;
  sstream << "transform_listener_impl_" << std::hex << reinterpret_cast<size_t>(this);
  rclcpp::NodeOptions options;
  // but specify its name in .arguments to override any __node passed on the command line
  options.arguments({ "--ros-args", "-r", "__node:=" + std::string(sstream.str()) });
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);
  auto node = rclcpp::Node::make_shared("_", options);

  
  auto tfl = std::make_shared<tf2_ros::TransformListener>(
      buffer, std::move(node), true, tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(),
      tf2_ros::detail::get_default_transform_listener_sub_options<std::allocator<void>>(),
      tf2_ros::detail::get_default_transform_listener_static_sub_options<std::allocator<void>>(),
       tf2_ros::TransformListener::ThreadFactory([future = std::shared_future(promise.get_future())](std::function<void()> fun) {
        return new std::thread([inner_future = std::move(future), inner_fun = std::move(fun)]() {
          inner_future.wait();
          inner_fun();
        });
      }
  ));

  auto tfl_future = std::async(std::launch::async, [](std::shared_ptr<tf2_ros::TransformListener> pointer){
    pointer.reset(); // call destructor, it should not block
  }, std::move(tfl));

  // This timeout is not perfect but otherwise we would need to inject the signalling to dedicated thread destructor
  auto status = tfl_future.wait_for(std::chrono::milliseconds(100));
  ASSERT_EQ(status, std::future_status::timeout);

  // This ensures that the tf2_ros::TransformListener destructor has been called before the
  // dedicated thread was able to run. The next line allows the inner thread to run
  promise.set_value();

  status = tfl_future.wait_for(std::chrono::milliseconds(100));
  ASSERT_EQ(status, std::future_status::ready);

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
