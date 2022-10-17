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

#include "keyboard_to_joy/keyboard_to_joy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <signal.h>
#include <stdio.h>

#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_U 0x75
#define KEYCODE_V 0x76

KeyboardReader::KeyboardReader() {
  tcgetattr(0, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(0, TCSANOW, &raw);
}

void KeyboardReader::readOne(char *c) {
  int rc = read(0, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}
KeyboardReader::~KeyboardReader() { tcsetattr(0, TCSANOW, &cooked); }

Keyboard2Joy::Keyboard2Joy(const rclcpp::NodeOptions &node_options)
    : Node("keyboard2joy", node_options) {
  joy = std::make_shared<sensor_msgs::msg::Joy>();
  joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 1);

  up_ = this->declare_parameter<int>("up");
  down_ = this->declare_parameter<int>("down");
  left_ = this->declare_parameter<int>("left");
  right_ = this->declare_parameter<int>("right");

  l1_ = this->declare_parameter<int>("l1");
  l2_ = this->declare_parameter<int>("l2");
  r1_ = this->declare_parameter<int>("r1");
  r2_ = this->declare_parameter<int>("r2");

  axes_ = this->declare_parameter<int>("axes");
  buttons_ = this->declare_parameter<int>("buttons");
  x_a_ = this->declare_parameter<int>("x_a");
  o_b_ = this->declare_parameter<int>("o_b");
  square_x_ = this->declare_parameter<int>("square_x");
  triangle_y_ = this->declare_parameter<int>("triangle_y");
  share_ = this->declare_parameter<int>("share");
  options_ = this->declare_parameter<int>("options");
  ps_ = this->declare_parameter<int>("ps");

  KeyboardReader input;
  keyLoop(input);
}

Keyboard2Joy::~Keyboard2Joy() {
  rclcpp::shutdown();
  exit(0);
}

void Keyboard2Joy::keyLoop(KeyboardReader input) {
  RCLCPP_WARN(this->get_logger(), "Keyboard to Joy starting");
  char c;

  joy->axes.resize(axes_);
  joy->buttons.resize(buttons_);

  RCLCPP_INFO(this->get_logger(),
              "=====================    KEYBOARD MAP   =====================");
  RCLCPP_INFO(this->get_logger(),
              " Shift UP  :   [u]            | Shift DOWN:   [d]");
  RCLCPP_INFO(this->get_logger(),
              " Vehicle Engage : [v]         | Vehicle Disengage:  [v]");
  RCLCPP_INFO(this->get_logger(),
              " Autoware Engage: [a]         | Autoware Disengage: [a]");
  RCLCPP_INFO(this->get_logger(),
              " Set Emergency Stop:  [e]     | Clear Emergency Stop:  [e]");
  RCLCPP_INFO(this->get_logger(),
              " Turn Signal LEFT: [LEFT]     | Turn Signal RIGHT:   [RIGHT]");
  RCLCPP_INFO(this->get_logger(),
              " Accel/Speed UP:  [UP]        | Brake/Speed DOWN:    [DOWN]");
  RCLCPP_INFO(this->get_logger(),
              " Steer LEFT:      [LEFT]      |  Steer RIGHT:        [RIGHT]");
  RCLCPP_INFO(this->get_logger(),
              " Exit: Ctrl + "
              "C　　　　　　　　　　　　　　　　　　　　　　　　　　　　　");
  RCLCPP_INFO(this->get_logger(),
              "===========================================================\n");

  while (true) {
    try {
      input.readOne(&c);
    } catch (const std::runtime_error &) {
      perror("read():");
      return;
    }

    // RCLCPP_INFO(get_logger(), "value: 0x%02X\n", c);

    for (size_t i = 0; i < joy->axes.size(); i++) {
      joy->axes[i] = 0;
    }
    for (size_t i = 0; i < joy->buttons.size(); i++) {
      joy->buttons[i] = 0;
    }

    bool is_pub_ = true;
    switch (c) {
    case KEYCODE_LEFT:
      RCLCPP_INFO(get_logger(), "LEFT");
      joy->axes[left_] = 1;
      break;
    case KEYCODE_RIGHT:
      RCLCPP_INFO(get_logger(), "RIGHT");
      joy->axes[right_] = -1;
      break;
    case KEYCODE_UP:
      RCLCPP_INFO(get_logger(), "UP");
      joy->axes[up_] = 1;
      break;
    case KEYCODE_DOWN:
      RCLCPP_INFO(get_logger(), "DOWN");
      joy->axes[down_] = -1;
      break;
    case KEYCODE_A:
      RCLCPP_INFO(get_logger(), "A");
      joy->buttons[square_x_] = 1;
      break;
    case KEYCODE_D:
      RCLCPP_INFO(get_logger(), "D");
      joy->buttons[o_b_] = 1;
      break;
    case KEYCODE_E:
      RCLCPP_INFO(get_logger(), "E");
      joy->buttons[r1_] = 1;
      break;
    case KEYCODE_U:
      RCLCPP_INFO(get_logger(), "U");
      joy->buttons[l1_] = 1;
      break;
    case KEYCODE_V:
      RCLCPP_INFO(get_logger(), "V");
      joy->buttons[x_a_] = 1;
      break;
    default:
      is_pub_ = false;
      break;
    }

    if (is_pub_) {
      joy_pub_->publish(*joy);
    }
  }
  return;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Keyboard2Joy)