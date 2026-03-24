// Copyright (c) 2024 Franka Robotics GmbH
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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <franka_hardware/franka_multi_hardware_interface_utils.hpp>

using franka_hardware::multi_interface_utils::BuildJointMapStrict;
using franka_hardware::multi_interface_utils::JointMap;
using franka_hardware::multi_interface_utils::ParseJointIndexStrict;
using franka_hardware::multi_interface_utils::ValidateNameStems;

TEST(MultiHardwareInterfaceUtilsTest, validateNameStems_duplicate) {
  std::vector<std::string> stems{"left_fr3_", "left_fr3_"};
  std::string error;
  EXPECT_FALSE(ValidateNameStems(stems, error));
  EXPECT_FALSE(error.empty());
}

TEST(MultiHardwareInterfaceUtilsTest, validateNameStems_prefixCollision) {
  std::vector<std::string> stems{"left_fr3_", "left_fr3_v2_"};
  std::string error;
  EXPECT_FALSE(ValidateNameStems(stems, error));
  EXPECT_FALSE(error.empty());
}

TEST(MultiHardwareInterfaceUtilsTest, buildJointMapStrict_success) {
  std::vector<std::string> stems{"left_fr3_", "right_fr3_"};
  std::vector<std::string> joint_names;
  for (const auto& stem : stems) {
    for (int i = 1; i <= 7; ++i) {
      joint_names.push_back(stem + "joint" + std::to_string(i));
    }
  }

  JointMap map;
  std::string error;
  EXPECT_TRUE(BuildJointMapStrict(stems, joint_names, map, error));
  EXPECT_EQ(map.size(), joint_names.size());
}

TEST(MultiHardwareInterfaceUtilsTest, buildJointMapStrict_missingJoint) {
  std::vector<std::string> stems{"left_fr3_"};
  std::vector<std::string> joint_names{
      "left_fr3_joint1", "left_fr3_joint2", "left_fr3_joint3",
      "left_fr3_joint4", "left_fr3_joint5", "left_fr3_joint6"};

  JointMap map;
  std::string error;
  EXPECT_FALSE(BuildJointMapStrict(stems, joint_names, map, error));
  EXPECT_FALSE(error.empty());
}

TEST(MultiHardwareInterfaceUtilsTest, buildJointMapStrict_invalidJoint) {
  std::vector<std::string> stems{"left_fr3_"};
  std::vector<std::string> joint_names{"left_fr3_joint1", "left_fr3_jointX"};

  JointMap map;
  std::string error;
  EXPECT_FALSE(BuildJointMapStrict(stems, joint_names, map, error));
  EXPECT_FALSE(error.empty());
}

TEST(MultiHardwareInterfaceUtilsTest, buildJointMapStrict_duplicateJoint) {
  std::vector<std::string> stems{"left_fr3_"};
  std::vector<std::string> joint_names{"left_fr3_joint1", "left_fr3_joint1", "left_fr3_joint2",
                                       "left_fr3_joint3", "left_fr3_joint4", "left_fr3_joint5",
                                       "left_fr3_joint6", "left_fr3_joint7"};

  JointMap map;
  std::string error;
  EXPECT_FALSE(BuildJointMapStrict(stems, joint_names, map, error));
  EXPECT_FALSE(error.empty());
}

TEST(MultiHardwareInterfaceUtilsTest, parseJointIndexStrict_multipleStemMatch) {
  std::vector<std::string> stems{"fr3_", "fr3_v2_"};
  size_t robot_idx = 0;
  size_t joint_idx = 0;
  std::string error;
  EXPECT_FALSE(ParseJointIndexStrict("fr3_v2_joint1", stems, robot_idx, joint_idx, error));
  EXPECT_FALSE(error.empty());
}
