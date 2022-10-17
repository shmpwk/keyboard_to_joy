// Copyright 2022 Tier IV, Inc.
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

#ifndef KEYBOARD2JOY__KEYBOARD2JOY_HPP_
#define KEYBOARD2JOY__KEYBOARD2JOY_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <termios.h>

class KeyboardReader {
public:
  KeyboardReader();
  void readOne(char *c);
  void shutdown();
  ~KeyboardReader();

private:
  struct termios cooked;
};

class Keyboard2Joy : public rclcpp::Node {
public:
  explicit Keyboard2Joy(const rclcpp::NodeOptions &node_options);
  ~Keyboard2Joy();

private:
  void keyLoop(KeyboardReader input);

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  std::shared_ptr<sensor_msgs::msg::Joy> joy;

  int axes_, buttons_, left_, right_, up_, down_, x_a_, o_b_, square_x_,
      triangle_y_, r1_, r2_, l1_, l2_, share_, options_, ps_;
};

#endif // KEYBOARD2JOY__KEYBOARD2JOY_