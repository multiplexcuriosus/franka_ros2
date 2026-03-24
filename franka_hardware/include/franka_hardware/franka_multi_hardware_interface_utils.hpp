#pragma once

#include <array>
#include <cctype>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace franka_hardware {
namespace multi_interface_utils {

constexpr size_t kNumberOfJoints = 7;

using JointIndex = std::pair<size_t, size_t>;  // (robot_idx, joint_idx)
using JointMap = std::unordered_map<std::string, JointIndex>;

inline bool ValidateNameStems(const std::vector<std::string>& stems, std::string& error_message) {
  if (stems.empty()) {
    error_message = "no robots configured";
    return false;
  }

  std::unordered_set<std::string> seen;
  for (size_t i = 0; i < stems.size(); ++i) {
    const auto& stem = stems[i];
    if (stem.empty()) {
      error_message = "name_stem cannot be empty";
      return false;
    }
    if (!seen.insert(stem).second) {
      error_message = "duplicate name_stem '" + stem + "'";
      return false;
    }
  }

  for (size_t i = 0; i < stems.size(); ++i) {
    const auto& a = stems[i];
    for (size_t j = 0; j < stems.size(); ++j) {
      if (i == j) {
        continue;
      }
      const auto& b = stems[j];
      if (b.rfind(a, 0) == 0) {
        error_message = "name_stem prefix collision: '" + a + "' is a prefix of '" + b + "'";
        return false;
      }
    }
  }

  return true;
}

inline bool ParseJointIndexStrict(const std::string& full_joint_name,
                                  const std::vector<std::string>& stems,
                                  size_t& robot_idx_out,
                                  size_t& joint_idx_out,
                                  std::string& error_message) {
  size_t matched_robot_idx = static_cast<size_t>(-1);
  size_t match_count = 0;
  size_t matched_stem_length = 0;

  for (size_t candidate_robot_idx = 0; candidate_robot_idx < stems.size(); ++candidate_robot_idx) {
    const auto& stem = stems[candidate_robot_idx];
    if (stem.empty()) {
      continue;
    }
    if (full_joint_name.rfind(stem, 0) == 0) {
      matched_robot_idx = candidate_robot_idx;
      ++match_count;
      matched_stem_length = stem.size();
    }
  }

  if (match_count == 0) {
    error_message = "does not match any robot name_stem";
    return false;
  }
  if (match_count > 1) {
    error_message =
        "matches " + std::to_string(match_count) + " robot name_stems (multiple robots match this joint name)";
    return false;
  }

  robot_idx_out = matched_robot_idx;

  const std::string suffix_after_stem = full_joint_name.substr(matched_stem_length);
  const std::string joint_tag = "joint";
  if (suffix_after_stem.rfind(joint_tag, 0) != 0) {
    error_message = "after stem, expected 'jointN' but got '" + suffix_after_stem + "'";
    return false;
  }

  const std::string joint_idx_str = suffix_after_stem.substr(joint_tag.size());
  if (joint_idx_str.empty()) {
    error_message = "missing joint number after 'joint'";
    return false;
  }

  for (char c : joint_idx_str) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      error_message = "joint number '" + joint_idx_str + "' contains non-numeric characters";
      return false;
    }
  }

  int joint_idx = 0;
  try {
    joint_idx = std::stoi(joint_idx_str);
  } catch (...) {
    error_message = "failed to parse joint number '" + joint_idx_str + "'";
    return false;
  }

  if (joint_idx < 1 || joint_idx > static_cast<int>(kNumberOfJoints)) {
    error_message = "joint number " + std::to_string(joint_idx) + " is out of valid range [1..7]";
    return false;
  }

  joint_idx_out = static_cast<size_t>(joint_idx - 1);
  return true;
}

inline bool BuildJointMapStrict(const std::vector<std::string>& stems,
                                const std::vector<std::string>& joint_names,
                                JointMap& joint_map_out,
                                std::string& error_message) {
  joint_map_out.clear();

  std::vector<std::array<bool, kNumberOfJoints>> seen(stems.size());
  for (auto& arr : seen) {
    arr.fill(false);
  }

  for (const auto& joint_name : joint_names) {
    size_t robot_idx = 0;
    size_t joint_idx = 0;
    if (!ParseJointIndexStrict(joint_name, stems, robot_idx, joint_idx, error_message)) {
      error_message = "joint '" + joint_name + "': " + error_message;
      return false;
    }

    if (seen[robot_idx][joint_idx]) {
      error_message =
          "duplicate joint mapping for joint" + std::to_string(joint_idx + 1) + " (name '" + joint_name + "')";
      return false;
    }

    seen[robot_idx][joint_idx] = true;
    joint_map_out[joint_name] = JointIndex{robot_idx, joint_idx};
  }

  for (size_t robot_idx = 0; robot_idx < stems.size(); ++robot_idx) {
    for (size_t joint_idx = 0; joint_idx < kNumberOfJoints; ++joint_idx) {
      if (!seen[robot_idx][joint_idx]) {
        error_message = "missing joint" + std::to_string(joint_idx + 1) + " for stem '" + stems[robot_idx] + "'";
        return false;
      }
    }
  }

  return true;
}

}  // namespace multi_interface_utils
}  // namespace franka_hardware
