#include "franka_hardware/franka_multi_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <sstream>
#include <stdexcept>

// parallel worker support
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

// RT priority support
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sched.h>

#include <fmt/core.h>
#include <franka/exception.h>
#include <franka/logging/logger.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "franka_hardware/ros_libfranka_logger.hpp"
#include "franka_hardware/franka_multi_hardware_interface_utils.hpp"

#include "pluginlib/class_list_macros.hpp"

const std::string kVersionName = "version";
const std::string kRobotCountName = "robot_count";

// robot_i fields
const std::string kFieldArmId = "arm_id";
const std::string kFieldRobotIp = "robot_ip";

namespace 
{
    bool isEnvEnabled(const char* name)
    {
        const char* raw = std::getenv(name);
        if (raw == nullptr) 
        {
            return false;
        }

        std::string value(raw);
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        return value == "1" || value == "true" || value == "on" || value == "yes";
    }

    const bool kTimingDiagEnabled = isEnvEnabled("FRANKA_MULTI_TIMING_DIAG");
    constexpr double kTimingWarnUs = 900.0;

    auto logRclcppFatalRed(const rclcpp::Logger& logger, const std::string& msg) 
    {
        RCLCPP_FATAL(logger, "\033[1;31m%s\033[0m", msg.c_str());
    }

    auto parseVersion(const std::string& version_str) 
    {
        std::vector<std::string> version_parts;
        std::stringstream ss(version_str);
        std::string item;
        while (std::getline(ss, item, '.')) 
        {
            version_parts.push_back(item);
        }

        if (version_parts.size() != 3) 
        {
            throw std::invalid_argument(
                "\033[1;31mInvalid version structure in URDF. Please update your URDF (aka "
                "franka_description).\033[0m");
        }

        return std::make_tuple(std::stoi(version_parts[0]), std::stoi(version_parts[1]),
                                std::stoi(version_parts[2]));
    }

}  // namespace

namespace franka_hardware 
{
    FrankaMultiHardwareInterface::FrankaMultiHardwareInterface()
    : command_interfaces_info_multi_({
        {hardware_interface::HW_IF_EFFORT,   kNumberOfJoints, [](RobotContext& rc, bool v){ rc.effort_claimed = v; }, [](const RobotContext& rc){ return rc.effort_claimed; }},
        {hardware_interface::HW_IF_VELOCITY, kNumberOfJoints, [](RobotContext& rc, bool v){ rc.jvel_claimed = v; },   [](const RobotContext& rc){ return rc.jvel_claimed; }},
        {hardware_interface::HW_IF_POSITION, kNumberOfJoints, [](RobotContext& rc, bool v){ rc.jpos_claimed = v; },   [](const RobotContext& rc){ return rc.jpos_claimed; }},
        {k_hw_if_elbow_command_,             2,               [](RobotContext& rc, bool v){ rc.elbow_claimed = v; },  [](const RobotContext& rc){ return rc.elbow_claimed; }},
        {k_hw_if_cartesian_velocity_,        6,               [](RobotContext& rc, bool v){ rc.cvel_claimed = v; },   [](const RobotContext& rc){ return rc.cvel_claimed; }},
        {k_hw_if_cartesian_pose_command_,    16,              [](RobotContext& rc, bool v){ rc.cpose_claimed = v; },  [](const RobotContext& rc){ return rc.cpose_claimed; }},
    })
    {
        // Allow libfranka to use the ROS logger
        franka::logging::addLogger(std::make_shared<RosLibfrankaLogger>(getLogger()));
    }

    // ---- Destructor: must join worker threads before members are destroyed ----
    FrankaMultiHardwareInterface::~FrankaMultiHardwareInterface()
    {
        stopWorkerThreads();
    }

    CallbackReturn FrankaMultiHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) 
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) 
        {
            return CallbackReturn::ERROR;
        }

        // Build set of exported command interfaces for direct lookup
        exported_command_interfaces_.clear();
        for (const auto& joint : info.joints) 
        {
            for (const auto& cmd_interface : joint.command_interfaces) 
            {
                exported_command_interfaces_.insert(joint.name + "/" + cmd_interface.name);
            }
        }
        for (const auto& gpio : info.gpios) 
        {
            for (const auto& cmd_interface : gpio.command_interfaces) 
            {
                exported_command_interfaces_.insert(gpio.name + "/" + cmd_interface.name);
            }
        }

        // ---- version check ----
        try 
        {
            const auto version_str = info_.hardware_parameters.at(kVersionName);
            auto [major, minor, patch] = parseVersion(version_str);
            RCLCPP_INFO(getLogger(), "Parsed Franka ros2_control interface version: %d.%d.%d", major, minor, patch);

            if (major != kSupportedControlInterfaceMajor_) 
            {
                logRclcppFatalRed(getLogger(), fmt::format(
                    "Unsupported major version of the Franka ros2_control interface. Expected %d, got %d. "
                    "Please update your URDF (aka franka_description).",
                    kSupportedControlInterfaceMajor_, major));
                return CallbackReturn::ERROR;
            }
        } 
        catch (const std::out_of_range& ex) 
        {
            logRclcppFatalRed(getLogger(), "Parameter 'version' is not set. Please update your URDF (aka franka_description).");
            return CallbackReturn::ERROR;
        } 
        catch (const std::exception& e) 
        {
            logRclcppFatalRed(getLogger(), e.what());
            return CallbackReturn::ERROR;
        }

        // ---- robot_count ----
        size_t robot_count = 0;
        try 
        {
            robot_count = static_cast<size_t>(std::stoul(info_.hardware_parameters.at(kRobotCountName)));
        } 
        catch (const std::out_of_range& ex) 
        {
            logRclcppFatalRed(getLogger(), "Parameter 'robot_count' is not set for multi interface.");
            return CallbackReturn::ERROR;
        } 
        catch (const std::exception& e) 
        {
            logRclcppFatalRed(getLogger(), fmt::format("Invalid 'robot_count': {}", e.what()));
            return CallbackReturn::ERROR;
        }

        if (robot_count == 0) 
        {
            logRclcppFatalRed(getLogger(), "robot_count must be > 1.");
            return CallbackReturn::ERROR;
        }
        else if (robot_count == 1) 
        {
            logRclcppFatalRed(getLogger(),
                              "FrankaMultiHardwareInterface requires robot_count > 1. "
                              "For a single robot, use FrankaHardwareInterface.");
            return CallbackReturn::ERROR;
        }

        robots_.clear();
        robots_.reserve(robot_count);

        // ---- parse per-robot params + connect ----
        for (size_t i = 0; i < robot_count; ++i) 
        {
            RobotContext rc;
            rc.arm_idx = static_cast<unsigned int>(i);

            // arm_id (required)
            const std::string key_arm = "robot_" + std::to_string(i) + "_" + kFieldArmId;
            try 
            {
                rc.arm_id = info_.hardware_parameters.at(key_arm);
            } 
            catch (const std::out_of_range&) 
            {
                logRclcppFatalRed(getLogger(), fmt::format("Missing required parameter '{}'", key_arm));
                return CallbackReturn::ERROR;
            }

            // prefix (optional)
            const std::string key_pre = "robot_" + std::to_string(i) + "_prefix";
            auto itp = info_.hardware_parameters.find(key_pre);
            rc.user_prefix = (itp != info_.hardware_parameters.end()) ? itp->second : "";

            // build stem:
            // if prefix given => "{prefix}_{arm_id}"
            // else            "{arm_id}"
            if (!rc.user_prefix.empty()) rc.name_stem = rc.user_prefix + "_" + rc.arm_id;
            else                         rc.name_stem = rc.arm_id;

            // robot_ip (required)
            const std::string key_ip = "robot_" + std::to_string(i) + "_" + kFieldRobotIp;
            try 
            {
                rc.robot_ip = info_.hardware_parameters.at(key_ip);
            } 
            catch (const std::out_of_range&) 
            {
                logRclcppFatalRed(getLogger(), fmt::format("Missing required parameter '{}'", key_ip));
                return CallbackReturn::ERROR;
            }

            // connect robot
            try 
            {
                RCLCPP_INFO(getLogger(), "Connecting robot[%zu] stem='%s' arm_id='%s' ip='%s' ...", i, rc.name_stem.c_str(), rc.arm_id.c_str(), rc.robot_ip.c_str());
                rc.robot = std::make_shared<Robot>(rc.robot_ip, getLogger());
                RCLCPP_INFO(getLogger(), "Successfully connected to robot");
            } 
            catch (const franka::Exception& e) 
            {
                logRclcppFatalRed(getLogger(), fmt::format("Could not connect robot[{}] at '{}': {}", i, rc.robot_ip, e.what()));
                return CallbackReturn::ERROR;
            }

            // nodes
            rc.service_node = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), rc.robot);
            rc.executor = std::make_shared<FrankaExecutor>();
            rc.executor->add_node(rc.service_node);

            rc.action_node = std::make_shared<ActionServer>(rclcpp::NodeOptions(), rc.robot);
            rc.executor->add_node(rc.action_node);

            robots_.push_back(std::move(rc));
            robots_.back().robot_state_ptr = &robots_.back().robot_state;
        }

        // ---- validate name_stem uniqueness + prefix collision ----
        std::vector<std::string> name_stems;
        name_stems.reserve(robots_.size());
        for (const auto& rc : robots_) 
        {
            name_stems.push_back(rc.name_stem);
        }

        std::string stem_error_message;
        if (!multi_interface_utils::ValidateNameStems(name_stems, stem_error_message)) 
        {
            logRclcppFatalRed(getLogger(), fmt::format(
                "Invalid name_stem configuration: {}", stem_error_message));
            return CallbackReturn::ERROR;
        }

        // ---- Build joint_map_ : each joint name must follow {stem}_joint[1..7] ----
        joint_map_.clear();

        std::vector<std::string> joint_name_stems;
        joint_name_stems.reserve(name_stems.size());
        for (const auto& stem : name_stems)
        {
            joint_name_stems.push_back(stem + "_");
        }

        std::vector<std::string> joint_names;
        joint_names.reserve(info_.joints.size());
        for (const auto& joint : info_.joints) 
        {
            joint_names.push_back(joint.name);
        }

        multi_interface_utils::JointMap parsed_joint_map;
        std::string joint_error_message;
        if (!multi_interface_utils::BuildJointMapStrict(joint_name_stems, joint_names, parsed_joint_map, joint_error_message)) 
        {
            std::string example_message;
            if (!robots_.empty()) 
            {
                example_message = fmt::format("Examples: '{}_joint1'..'{}_joint7'", robots_[0].name_stem, robots_[0].name_stem);
                if (robots_.size() >= 2) 
                {
                    example_message += fmt::format(" | '{}_joint1'..'{}_joint7'", robots_[1].name_stem, robots_[1].name_stem);
                }
            }
            logRclcppFatalRed(getLogger(), fmt::format(
                "Invalid joint name mapping: {}. Expected '{prefix}_{arm_id}_jointN' or '{arm_id}_jointN' with N in [1..7]. {}",
                joint_error_message, example_message));
            return CallbackReturn::ERROR;
        }

        for (const auto& entry : parsed_joint_map) 
        {
            joint_map_[entry.first] = JointIndex{entry.second.first, entry.second.second};
        }

        // ---- Validate GPIO index uniqueness per robot and interface type ----
        {
            std::vector<std::unordered_map<std::string, std::unordered_set<size_t>>> used_indices(robots_.size());

            auto expected_size = [this](const std::string& interface_name) -> size_t
            {
                if (interface_name == k_hw_if_cartesian_velocity_) return 6;
                if (interface_name == k_hw_if_cartesian_pose_command_) return 16;
                if (interface_name == k_hw_if_elbow_command_) return 2;
                return 0;
            };

            for (const auto& gpio : info_.gpios) 
            {
                size_t robot_idx = 0;
                if (!findRobotByResourceName(gpio.name, robot_idx)) 
                {
                    logRclcppFatalRed(getLogger(), fmt::format(
                        "GPIO '{}' does not match any robot prefix/arm_id. Fix URDF naming.",
                        gpio.name));
                    return CallbackReturn::ERROR;
                }

                size_t idx = 0;
                try 
                {
                    idx = static_cast<size_t>(std::stoul(gpio.parameters.at("index")));
                } 
                catch (...) 
                {
                    logRclcppFatalRed(getLogger(), fmt::format(
                        "GPIO '{}' missing numeric parameter 'index'.", gpio.name));
                    return CallbackReturn::ERROR;
                }

                for (const auto& cmd : gpio.command_interfaces) 
                {
                    auto& set = used_indices[robot_idx][cmd.name];
                    if (!set.insert(idx).second) 
                    {
                        logRclcppFatalRed(getLogger(), fmt::format(
                            "Duplicate GPIO index {} for robot[%zu] interface '{}'. Fix URDF indices.",
                            idx, robot_idx, cmd.name));
                        return CallbackReturn::ERROR;
                    }

                    const size_t max_size = expected_size(cmd.name);
                    if (max_size != 0U && idx >= max_size) 
                    {
                        logRclcppFatalRed(getLogger(), fmt::format(
                            "GPIO '{}' index {} out of range for interface '{}' (size={}).",
                            gpio.name, idx, cmd.name, max_size));
                        return CallbackReturn::ERROR;
                    }
                }
            }
        }

        RCLCPP_INFO(getLogger(),
                    "FrankaMultiHardwareInterface initialized with %zu robots. Joint mapping validated strictly.",
                    robots_.size());
        if (kTimingDiagEnabled)
        {
            RCLCPP_WARN(getLogger(),
                        "[timing_diag] FRANKA_MULTI_TIMING_DIAG is enabled; read/write loop timing warnings are active.");
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn FrankaMultiHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
    {
        // Pre-initialize model pointers so the first read worker doesn't pay the cost
        for (auto& rc : robots_)
        {
            if (rc.model_ptr == nullptr)
            {
                rc.model_ptr = rc.robot->getModel();
            }
        }

        read(rclcpp::Time(0), rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
        for (auto& rc : robots_) 
        {
            const bool comm_violation =
                rc.robot_state.current_errors.communication_constraints_violation ||
                rc.robot_state.last_motion_errors.communication_constraints_violation;
            if (comm_violation && !rc.comm_constraints_recovery_sent)
            {
                RCLCPP_WARN(getLogger(),
                            "communication_constraints_violation detected for '%s'; sending automatic error recovery.",
                            rc.name_stem.c_str());
                try 
                {
                    rc.robot->automaticErrorRecovery();
                }
                catch (const franka::Exception& e)
                {
                    RCLCPP_ERROR(getLogger(), "Automatic error recovery failed for '%s': %s",
                                 rc.name_stem.c_str(), e.what());
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(getLogger(), "Automatic error recovery failed for '%s': %s",
                                 rc.name_stem.c_str(), e.what());
                }
                rc.comm_constraints_recovery_sent = true;
            }
        }
        RCLCPP_INFO(getLogger(), "Started");
        startWorkerThreads();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn FrankaMultiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
    {
        RCLCPP_INFO(getLogger(), "Stopping all robots...");
        stopWorkerThreads();  // join workers BEFORE stopping robots to avoid use-after-stop
        for (auto& rc : robots_) 
        {
            if (rc.robot) 
            {
                RCLCPP_INFO(getLogger(), "trying to Stop [%u] robot ...", rc.arm_idx);
                rc.robot->stopRobot();
            }
            rc.comm_constraints_recovery_sent = false;
        }
        RCLCPP_INFO(getLogger(), "Stopped");
        return CallbackReturn::SUCCESS;
    }

    // =========================================================================
    // Parallel worker thread helpers
    // =========================================================================

    // -------------------------------------------------------------------------
    // RT priority target: match the ros2_control RT loop default (50).
    // Workers must be >= RT loop priority so the OS never preempts a worker
    // while the RT loop thread is blocking on done_cv.wait().
    // -------------------------------------------------------------------------
    // Workers must be >= the RT loop priority so the RT loop cannot preempt
    // a worker while main is blocking in done_cv.wait().
    // ros2_control RT loop default = 98; workers use 99 (highest SCHED_FIFO).
    static constexpr int kWorkerRtPriority = 99;

    // Helper: read back and log the actual scheduler policy + priority of a thread.
    static void logThreadSchedulerInfo(const rclcpp::Logger& logger,
                                       pthread_t handle,
                                       size_t robot_idx,
                                       const std::string& robot_name)
    {
        int policy = 0;
        sched_param actual{};
        if (pthread_getschedparam(handle, &policy, &actual) == 0)
        {
            const char* policy_str =
                (policy == SCHED_FIFO)  ? "SCHED_FIFO"  :
                (policy == SCHED_RR)    ? "SCHED_RR"    :
                (policy == SCHED_OTHER) ? "SCHED_OTHER" : "UNKNOWN";
            RCLCPP_INFO(logger,
                "[RT] Worker[%zu] '%s': policy=%s, priority=%d",
                robot_idx, robot_name.c_str(), policy_str, actual.sched_priority);
        }
        else
        {
            RCLCPP_WARN(logger,
                "[RT] Worker[%zu] '%s': pthread_getschedparam failed (errno=%d: %s)",
                robot_idx, robot_name.c_str(), errno, strerror(errno));
        }
    }

    void FrankaMultiHardwareInterface::startWorkerThreads()
    {
        if (!worker_threads_.empty())
        {
            RCLCPP_DEBUG(getLogger(), "[RT] startWorkerThreads() called but threads already running — skipped.");
            return;  // already running (idempotent)
        }

        // ---- Log the calling thread's scheduler info (= ros2_control RT loop) ----
        {
            int policy = 0;
            sched_param param{};
            if (pthread_getschedparam(pthread_self(), &policy, &param) == 0)
            {
                const char* policy_str =
                    (policy == SCHED_FIFO)  ? "SCHED_FIFO"  :
                    (policy == SCHED_RR)    ? "SCHED_RR"    :
                    (policy == SCHED_OTHER) ? "SCHED_OTHER" : "UNKNOWN";
                RCLCPP_INFO(getLogger(),
                    "[RT] on_activate caller thread: policy=%s, priority=%d  "
                    "→ workers will target SCHED_FIFO priority=%d",
                    policy_str, param.sched_priority, kWorkerRtPriority);

                // Warn if workers would be LOWER priority than this caller.
                // Note: on_activate is called from a high-priority lifecycle thread (often 99),
                // but the RT control loop (read/write) typically runs at 98.
                // Workers at 99 are safe: they beat the RT loop (98) so the loop
                // cannot preempt a worker while main is in done_cv.wait().
                if (kWorkerRtPriority < param.sched_priority && policy == SCHED_FIFO)
                {
                    RCLCPP_WARN(getLogger(),
                        "[RT] WARNING: worker priority (%d) < on_activate caller priority (%d). "
                        "This is usually OK if the RT loop runs lower than this thread. "
                        "Verify controller_manager RT priority in the log below.",
                        kWorkerRtPriority, param.sched_priority);
                }
            }
        }

        worker_threads_.reserve(robots_.size());

        for (size_t i = 0; i < robots_.size(); ++i)
        {
            const std::string robot_name = robots_[i].name_stem;  // capture by value for lambda

            // ---- Spawn worker thread ----
            // The worker handles one full readOnce()+writeOnce() cycle per wake-up.
            // Merging READ and WRITE into a single CYCLE command halves the number of
            // mutex lock/unlock pairs per 1 kHz tick and keeps latency predictable.
            worker_threads_.emplace_back([this, i, robot_name]()
            {
                RobotContext& rc = robots_[i];

                while (true)
                {
                    // ---- Wait for CYCLE or SHUTDOWN from main thread ----
                    RobotContext::WorkerCmd cmd;
                    {
                        std::unique_lock<std::mutex> lk(rc.sync->cmd_mtx);
                        rc.sync->cmd_cv.wait(lk, [&rc]
                        {
                            return rc.sync->cmd != RobotContext::WorkerCmd::IDLE;
                        });
                        cmd = rc.sync->cmd;
                        rc.sync->cmd = RobotContext::WorkerCmd::IDLE;  // consume immediately
                    }

                    if (cmd == RobotContext::WorkerCmd::SHUTDOWN)
                    {
                        RCLCPP_DEBUG(getLogger(),
                            "[RT] Worker[%zu] '%s': received SHUTDOWN, exiting loop.",
                            i, robot_name.c_str());
                        break;
                    }

                    // ---- CYCLE: readOnce then writeOnce in the same 1 ms window ----
                    // (The only non-SHUTDOWN command is CYCLE, so no further branch needed.)
                    rc.sync->cycle_result = hardware_interface::return_type::OK;
                    try
                    {
                        // Step 1: read current robot state into pending buffer.
                        //   Main thread commits pending_state → robot_state AFTER all
                        //   workers finish, so no race with the state interfaces.
                        rc.sync->pending_state = rc.robot->readOnce();

                        // Step 2: write commands that the controller filled during
                        //   the previous update() call (cmd_effort/cmd_pos/etc.).
                        //   readOnce() already advanced libfranka's internal clock,
                        //   so writeOnce() is within the same 1 ms budget.
                        rc.sync->cycle_result = writeOneRobot(rc);

                        rc.sync->worker_exception = nullptr;
                    }
                    catch (...)
                    {
                        rc.sync->worker_exception = std::current_exception();
                        rc.sync->cycle_result = hardware_interface::return_type::ERROR;
                    }

                    // ---- Signal cycle completion to main thread ----
                    {
                        std::lock_guard<std::mutex> lk(rc.sync->done_mtx);
                        rc.sync->done = true;
                    }
                    rc.sync->done_cv.notify_one();
                }
            });

            // ---- Set RT priority on the newly created thread ----
            pthread_t native = worker_threads_.back().native_handle();

            // Step 1: query the max priority allowed for SCHED_FIFO on this system
            const int max_fifo_priority = sched_get_priority_max(SCHED_FIFO);
            const int min_fifo_priority = sched_get_priority_min(SCHED_FIFO);

            if (max_fifo_priority < 0 || min_fifo_priority < 0)
            {
                // sched_get_priority_max/min returns -1 on error (e.g. unsupported policy)
                RCLCPP_ERROR(getLogger(),
                    "[RT] Worker[%zu] '%s': sched_get_priority_max(SCHED_FIFO) failed "
                    "(errno=%d: %s). RT priority NOT set. "
                    "communication_constraints_violation risk is elevated.",
                    i, robot_name.c_str(), errno, strerror(errno));
                logThreadSchedulerInfo(getLogger(), native, i, robot_name);
                continue;
            }

            // Step 2: clamp requested priority to system limits
            const int target_priority = std::max(min_fifo_priority,
                                                  std::min(kWorkerRtPriority, max_fifo_priority));
            if (target_priority != kWorkerRtPriority)
            {
                RCLCPP_WARN(getLogger(),
                    "[RT] Worker[%zu] '%s': requested priority=%d clamped to system limit [%d, %d] → using %d.",
                    i, robot_name.c_str(), kWorkerRtPriority,
                    min_fifo_priority, max_fifo_priority, target_priority);
            }

            // Step 3: apply SCHED_FIFO with the target priority
            sched_param rt_param{};
            rt_param.sched_priority = target_priority;
            const int ret = pthread_setschedparam(native, SCHED_FIFO, &rt_param);

            if (ret != 0)
            {
                // Common failure reasons:
                //   EPERM  (1) : process lacks CAP_SYS_NICE or rtprio limit in /etc/security/limits.conf
                //   EINVAL (22): invalid policy or priority value
                const char* hint =
                    (ret == EPERM)
                    ? "Check /etc/security/limits.conf (rtprio limit) and that the user is in the 'realtime' group."
                    : "Unexpected error — check system RT configuration.";

                RCLCPP_ERROR(getLogger(),
                    "[RT] Worker[%zu] '%s': pthread_setschedparam(SCHED_FIFO, priority=%d) FAILED "
                    "(errno=%d: %s). %s "
                    "Worker will run as SCHED_OTHER. "
                    "communication_constraints_violation risk is elevated under CPU load.",
                    i, robot_name.c_str(), target_priority, ret, strerror(ret), hint);

                // Log the fallback (actual) scheduler state so the user knows what they got
                logThreadSchedulerInfo(getLogger(), native, i, robot_name);
            }
            else
            {
                RCLCPP_INFO(getLogger(),
                    "[RT] Worker[%zu] '%s': SCHED_FIFO priority=%d set successfully.",
                    i, robot_name.c_str(), target_priority);

                // Verify by reading back — confirms the kernel actually applied it
                logThreadSchedulerInfo(getLogger(), native, i, robot_name);
            }
        }

        RCLCPP_INFO(getLogger(),
            "[RT] All %zu parallel worker threads started. "
            "RT setup summary logged above — check for any ERROR/WARN lines.",
            robots_.size());
    }

    void FrankaMultiHardwareInterface::stopWorkerThreads()
    {
        if (worker_threads_.empty())
        {
            RCLCPP_DEBUG(getLogger(), "[RT] stopWorkerThreads() called but no threads running — skipped.");
            return;
        }

        RCLCPP_INFO(getLogger(), "[RT] Sending SHUTDOWN to %zu worker threads...", robots_.size());

        for (size_t i = 0; i < robots_.size(); ++i)
        {
            auto& rc = robots_[i];
            {
                std::lock_guard<std::mutex> lk(rc.sync->cmd_mtx);
                rc.sync->cmd = RobotContext::WorkerCmd::SHUTDOWN;
            }
            rc.sync->cmd_cv.notify_one();
            RCLCPP_DEBUG(getLogger(), "[RT] Worker[%zu] '%s': SHUTDOWN signal sent.",
                i, rc.name_stem.c_str());
        }

        for (size_t i = 0; i < worker_threads_.size(); ++i)
        {
            if (worker_threads_[i].joinable())
            {
                RCLCPP_DEBUG(getLogger(), "[RT] Joining worker[%zu] '%s'...",
                    i, robots_[i].name_stem.c_str());
                worker_threads_[i].join();
                RCLCPP_DEBUG(getLogger(), "[RT] Worker[%zu] '%s': joined.",
                    i, robots_[i].name_stem.c_str());
            }
        }
        worker_threads_.clear();
        RCLCPP_INFO(getLogger(), "[RT] All parallel worker threads stopped and joined.");
    }

    // =========================================================================
    // dispatchCycleAndWait
    //   Triggers one READ+WRITE cycle on every worker simultaneously,
    //   then blocks until all workers signal completion.
    //   Returns false if any worker threw (exception already logged).
    // =========================================================================
    bool FrankaMultiHardwareInterface::dispatchCycleAndWait()
    {
        // ── Phase 1: arm all workers ────────────────────────────────────────
        for (auto& rc : robots_)
        {
            std::lock_guard<std::mutex> lk(rc.sync->done_mtx);
            rc.sync->done = false;
            rc.sync->worker_exception = nullptr;
        }

        // ── Phase 2: fire all workers simultaneously ────────────────────────
        // Dispatch is done AFTER arming done=false so workers cannot finish
        // and signal before main starts waiting.
        for (auto& rc : robots_)
        {
            {
                std::lock_guard<std::mutex> lk(rc.sync->cmd_mtx);
                rc.sync->cmd = RobotContext::WorkerCmd::CYCLE;
            }
            rc.sync->cmd_cv.notify_one();
        }

        // ── Phase 3: wait for all workers ──────────────────────────────────
        bool any_exception = false;
        for (auto& rc : robots_)
        {
            std::unique_lock<std::mutex> lk(rc.sync->done_mtx);
            rc.sync->done_cv.wait(lk, [&rc]{ return rc.sync->done; });

            if (rc.sync->worker_exception)
            {
                any_exception = true;
                try { std::rethrow_exception(rc.sync->worker_exception); }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(getLogger(),
                        "[RT] Worker '%s' threw: %s", rc.name_stem.c_str(), e.what());
                }
                catch (...)
                {
                    RCLCPP_ERROR(getLogger(),
                        "[RT] Worker '%s' threw unknown exception", rc.name_stem.c_str());
                }
            }
        }

        // ── Phase 4: commit state (only if no exception) ────────────────────
        if (!any_exception)
        {
            for (auto& rc : robots_)
            {
                rc.robot_state      = rc.sync->pending_state;
                rc.robot_time_state = rc.robot_state.time.toSec();
                initializeCommandsFromState(rc);
                rc.q     = rc.robot_state.q;
                rc.dq    = rc.robot_state.dq;
                rc.tau   = rc.robot_state.tau_J;
                rc.elbow = rc.robot_state.elbow;
                rc.O_T_EE= rc.robot_state.O_T_EE;
            }
        }

        return !any_exception;
    }

    // =========================================================================
    // read() — kicks off parallel CYCLE (read+write merged)
    //   ros2_control calls read() then write() every tick.
    //   We fire the workers in read() and just harvest results in write()
    //   to avoid two separate round-trips per 1 ms tick.
    // =========================================================================
    hardware_interface::return_type FrankaMultiHardwareInterface::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // ── Fallback: no workers yet (bootstrap call in on_activate) ─────────
        if (worker_threads_.empty())
        {
            for (auto& rc : robots_)
            {
                rc.robot_state      = rc.robot->readOnce();
                rc.robot_time_state = rc.robot_state.time.toSec();
                initializeCommandsFromState(rc);
                rc.q     = rc.robot_state.q;
                rc.dq    = rc.robot_state.dq;
                rc.tau   = rc.robot_state.tau_J;
                rc.elbow = rc.robot_state.elbow;
                rc.O_T_EE= rc.robot_state.O_T_EE;
            }
            return hardware_interface::return_type::OK;
        }

        // ── Normal path: fire workers and immediately wait ────────────────────
        // read() and write() are separate ros2_control calls, but we collapse
        // them into one worker round-trip here.  write() becomes a no-op.
        const auto t0 =
            kTimingDiagEnabled ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};

        const bool ok = dispatchCycleAndWait();

        if (kTimingDiagEnabled)
        {
            const auto now = std::chrono::steady_clock::now();
            const double us = std::chrono::duration<double, std::micro>(now - t0).count();
            static auto last_warn = std::chrono::steady_clock::time_point{};
            static bool warned = false;
            if (us > kTimingWarnUs && (!warned || (now - last_warn) > std::chrono::milliseconds(500)))
            {
                RCLCPP_WARN(getLogger(),
                    "[timing_diag] parallel cycle (read+write) total=%.1f us "
                    "(threshold %.1f us, N=%zu robots)",
                    us, kTimingWarnUs, robots_.size());
                warned = true;
                last_warn = now;
            }
        }

        // Store result so write() can return it
        last_cycle_ok_ = ok;
        return ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
    }

    // =========================================================================
    // write() — workers already finished in read(); just check stored result
    // =========================================================================
    hardware_interface::return_type FrankaMultiHardwareInterface::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Fallback: workers not running
        if (worker_threads_.empty())
        {
            for (auto& rc : robots_)
            {
                const auto ret = writeOneRobot(rc);
                if (ret != hardware_interface::return_type::OK)
                    return ret;
            }
            return hardware_interface::return_type::OK;
        }

        // Normal path: cycle already completed in read().
        // Check per-robot write results.
        if (!last_cycle_ok_)
            return hardware_interface::return_type::ERROR;

        for (auto& rc : robots_)
        {
            if (rc.sync->cycle_result != hardware_interface::return_type::OK)
                return rc.sync->cycle_result;
        }

        return hardware_interface::return_type::OK;
    }


    std::vector<hardware_interface::StateInterface> FrankaMultiHardwareInterface::export_state_interfaces() 
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // joint states (from URDF joints)
        for (const auto& joint : info_.joints) 
        {
            const auto it = joint_map_.find(joint.name);
            if (it == joint_map_.end()) continue;
            const auto robot_idx = it->second.robot;
            const auto joint_idx = it->second.joint;

            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &robots_[robot_idx].q.at(joint_idx)));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &robots_[robot_idx].dq.at(joint_idx)));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT,   &robots_[robot_idx].tau.at(joint_idx)));
        }

        // per-robot special states: robot_state/model + cart pose state + elbow state + robot_time
        for (size_t r = 0; r < robots_.size(); ++r) 
        {
            auto& rc = robots_[r];

            state_interfaces.emplace_back(hardware_interface::StateInterface(rc.name_stem, k_robot_state_interface_name_, reinterpret_cast<double*>(&rc.robot_state_ptr)));  // NOLINT
            state_interfaces.emplace_back(hardware_interface::StateInterface(rc.name_stem, k_robot_model_interface_name_, reinterpret_cast<double*>(&rc.model_ptr)));  // NOLINT

            for (size_t i = 0; i < 16; ++i) 
            {                
                const std::string pose_name = rc.user_prefix + "_" + std::to_string(i);
                state_interfaces.emplace_back(hardware_interface::StateInterface(pose_name, k_hw_if_cartesian_pose_state_, &rc.O_T_EE.at(i)));
            }

            // elbow state: unique names
            state_interfaces.emplace_back(hardware_interface::StateInterface(rc.user_prefix + "_joint_3_position", k_hw_if_elbow_state_, &rc.elbow.at(0)));
            state_interfaces.emplace_back(hardware_interface::StateInterface(rc.user_prefix + "_joint_4_sign", k_hw_if_elbow_state_, &rc.elbow.at(1)));

            // robot time
            state_interfaces.emplace_back(hardware_interface::StateInterface(rc.name_stem, "robot_time", &rc.robot_time_state));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> FrankaMultiHardwareInterface::export_command_interfaces() 
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // joint-based commands from URDF
        RCLCPP_INFO(getLogger(), "Register joint-based command interfaces (multi)");
        for (const auto& joint : info_.joints) 
        {
            auto it = joint_map_.find(joint.name);
            if (it == joint_map_.end()) continue;
            const auto robot_idx = it->second.robot;
            const auto joint_idx = it->second.joint;

            for (const auto& cmd : joint.command_interfaces) 
            {
                if (cmd.name == hardware_interface::HW_IF_EFFORT)
                {
                    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, cmd.name, &robots_[robot_idx].cmd_effort.at(joint_idx)));
                }
                else if (cmd.name == hardware_interface::HW_IF_POSITION) 
                {
                    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, cmd.name, &robots_[robot_idx].cmd_pos.at(joint_idx)));
                }
                else if (cmd.name == hardware_interface::HW_IF_VELOCITY) 
                {
                    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, cmd.name, &robots_[robot_idx].cmd_vel.at(joint_idx)));
                }
                else 
                {
                    // Unknown joint command interface type
                    RCLCPP_WARN(getLogger(), "Unknown joint command interface '%s' for joint '%s' (ignored).",
                                cmd.name.c_str(), joint.name.c_str());
                }
            }
        }

        // GPIO-based commands (cartesian velocity / pose / elbow) from URDF
        // Expected: gpio.name is prefixed so we can map to robot; gpio.parameters["index"] selects element in vector
        RCLCPP_INFO(getLogger(), "Register gpio-based command interfaces (multi)");
        for (const auto& gpio : info_.gpios) 
        {
            size_t r = 0;
            if (!findRobotByResourceName(gpio.name, r)) 
            {
                RCLCPP_FATAL(getLogger(),
                            "GPIO '%s' does not match any robot prefix/arm_id. Fix URDF naming.",
                            gpio.name.c_str());
                continue;
            }

            size_t idx = 0;
            try 
            {
                idx = static_cast<size_t>(std::stoul(gpio.parameters.at("index")));
            } 
            catch (...) 
            {
                RCLCPP_FATAL(getLogger(), "GPIO '%s' missing numeric parameter 'index'.", gpio.name.c_str());
                continue;
            }

            for (const auto& cmd : gpio.command_interfaces) 
            {
                if (cmd.name == k_hw_if_cartesian_velocity_) 
                {
                    if (idx >= robots_[r].cmd_cartesian_vel.size()) 
                    {
                        RCLCPP_FATAL(getLogger(), "GPIO '%s' index %zu out of range for cart vel (size=%zu).",
                                    gpio.name.c_str(), idx, robots_[r].cmd_cartesian_vel.size());
                        continue;
                    }
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(gpio.name, cmd.name, &robots_[r].cmd_cartesian_vel.at(idx))
                    );
                } 
                else if (cmd.name == k_hw_if_cartesian_pose_command_) 
                {
                    if (idx >= robots_[r].cmd_cartesian_pose.size()) 
                    {
                        RCLCPP_FATAL(getLogger(), "GPIO '%s' index %zu out of range for cart pose (size=%zu).",
                                    gpio.name.c_str(), idx, robots_[r].cmd_cartesian_pose.size());
                        continue;
                    }
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(gpio.name, cmd.name, &robots_[r].cmd_cartesian_pose.at(idx))
                    );
                } else if (cmd.name == k_hw_if_elbow_command_) 
                {
                    if (idx >= robots_[r].cmd_elbow.size()) 
                    {
                        RCLCPP_FATAL(getLogger(), "GPIO '%s' index %zu out of range for elbow (size=%zu).",
                                    gpio.name.c_str(), idx, robots_[r].cmd_elbow.size());
                        continue;
                    }
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(gpio.name, cmd.name, &robots_[r].cmd_elbow.at(idx))
                    );
                } 
                else 
                {
                    RCLCPP_WARN(getLogger(), "Unknown gpio command interface '%s' for gpio '%s' (ignored).",
                                cmd.name.c_str(), gpio.name.c_str());
                }
            }
        }

        return command_interfaces;
        }

    hardware_interface::return_type FrankaMultiHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
    {
        // In multi, large start list can be valid, so scale the "too many" check
        const size_t max_all = max_number_start_interfaces_per_robot_ * robots_.size();
        if (start_interfaces.size() >= max_all) 
        {
            RCLCPP_FATAL(getLogger(),
                        "Invalid number of start interface. Do you return empty array in your controllers "
                        "command_interface_configuration?");
            return hardware_interface::return_type::ERROR;
        }

        auto contains_interface_type_for_robot = [this](const std::string& full_interface, const std::string& interface_type, size_t& robot_idx_out) -> bool
        {
            // 1) Must be an interface we actually export
            if (exported_command_interfaces_.count(full_interface) == 0) return false;

            // 2) Must contain "resource/type"
            const size_t slash_pos = full_interface.find('/');
            if (slash_pos == std::string::npos || slash_pos + 1 >= full_interface.size()) return false;

            const std::string resource_name = full_interface.substr(0, slash_pos);
            const std::string actual_type   = full_interface.substr(slash_pos + 1);

            if (actual_type != interface_type) return false;

            // 3) Map resource_name to a robot index (must be unique)
            return findRobotByResourceName(resource_name, robot_idx_out);
        };

        auto generate_error_message = [this](const std::string& start_stop_command, const std::string& interface_name, size_t robot_idx, size_t actual_interface_size, size_t expected_interface_size)
        {
            const std::string error_message = fmt::format(
                "Invalid number of {} interfaces to {} for robot[{}]. Expected {}, given {}",
                interface_name, start_stop_command, robot_idx, expected_interface_size, actual_interface_size);

            RCLCPP_FATAL(this->getLogger(), "%s", error_message.c_str());
            throw std::invalid_argument(error_message);
        };

        // Count start/stop interfaces per robot and per interface type
        std::vector<std::unordered_map<std::string, size_t>> stop_count(robots_.size());
        std::vector<std::unordered_map<std::string, size_t>> start_count(robots_.size());

        auto accumulate = [&](const std::vector<std::string>& interfaces, bool is_start)
        {
            for (const auto& full : interfaces) 
            {
                // We will test against each known interface type (small set), like single counts per type.
                for (const auto& interface : command_interfaces_info_multi_) 
                {
                    size_t robot_idx = 0;
                    if (contains_interface_type_for_robot(full, interface.interface_type, robot_idx)) 
                    {
                        if (robot_idx >= robots_.size()) continue; // defensive, should not happen

                        if (is_start) start_count[robot_idx][interface.interface_type]++;
                        else          stop_count[robot_idx][interface.interface_type]++;

                        break; // full interface can match only one type
                    }
                }
            }
        };

        accumulate(stop_interfaces, false);
        accumulate(start_interfaces, true);

        // Apply stop/start rules per robot
        for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) 
        {
            for (const auto& interface : command_interfaces_info_multi_) 
            {
                const size_t num_stop = stop_count[robot_idx].count(interface.interface_type) ? stop_count[robot_idx][interface.interface_type] : 0;
                const size_t num_start = start_count[robot_idx].count(interface.interface_type) ? start_count[robot_idx][interface.interface_type] : 0;

                if (num_stop == interface.size) interface.set_claimed(robots_[robot_idx], false);
                else if (num_stop != 0U)        generate_error_message("stop", interface.interface_type, robot_idx, num_stop, interface.size);

                if (num_start == interface.size) interface.set_claimed(robots_[robot_idx], true);
                else if (num_start != 0U)        generate_error_message("start", interface.interface_type, robot_idx, num_start, interface.size);
            }

            // Same rule as single: elbow cannot be claimed alone
            if (robots_[robot_idx].elbow_claimed &&  !(robots_[robot_idx].cvel_claimed || robots_[robot_idx].cpose_claimed))
            {
                RCLCPP_FATAL(getLogger(),
                            "Robot[%zu] elbow cannot be commanded without cartesian velocity or pose interface",
                            robot_idx);
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FrankaMultiHardwareInterface::perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) 
    {

        for (auto& rc : robots_) 
        {
            // effort
            if (!rc.effort_interface_running && rc.effort_claimed) 
            {
                // ── SAFE EFFORT INIT ──────────────────────────────────────
                // 0으로 초기화하면 gravity compensation 없이 로봇이 떨어짐.
                // 현재 robot_state의 gravity torque (tau_J - tau_ext)를 초기값으로
                // 사용하면 컨트롤러가 첫 번째 명령을 세팅하기 전까지 자세 유지.
                // robot_state.tau_J: measured joint torques (includes gravity)
                // robot_state.q가 유효한 경우에만 사용, 아니면 0
                if (rc.robot_state.q[0] != 0.0 || rc.robot_state.q[1] != 0.0)
                {
                    // tau_J는 현재 측정된 토크 (gravity 포함). 이를 초기 effort로 사용.
                    // 컨트롤러가 올바른 값을 세팅하기 전 1~2 사이클 동안 자세 유지.
                    for (size_t j = 0; j < rc.cmd_effort.size(); ++j)
                    {
                        rc.cmd_effort[j] = rc.robot_state.tau_J[j];
                    }
                    RCLCPP_INFO(getLogger(),
                        "Robot '%s': effort interface activated with gravity-compensating init torques "
                        "[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] Nm",
                        rc.name_stem.c_str(),
                        rc.cmd_effort[0], rc.cmd_effort[1], rc.cmd_effort[2], rc.cmd_effort[3],
                        rc.cmd_effort[4], rc.cmd_effort[5], rc.cmd_effort[6]);
                }
                else
                {
                    std::fill(rc.cmd_effort.begin(), rc.cmd_effort.end(), 0.0);
                    RCLCPP_WARN(getLogger(),
                        "Robot '%s': effort interface activated but robot_state not yet valid — "
                        "initializing to 0 torque. Ensure read() has been called first.",
                        rc.name_stem.c_str());
                }
                rc.robot->stopRobot();
                rc.robot->initializeTorqueInterface();
                rc.effort_interface_running = true;
            } 
            else if (rc.effort_interface_running && !rc.effort_claimed) 
            {
                rc.robot->stopRobot();
                rc.effort_interface_running = false;
            }

            // joint velocity
            if (!rc.jvel_interface_running && rc.jvel_claimed) 
            {
                std::fill(rc.cmd_vel.begin(), rc.cmd_vel.end(), 0.0);
                rc.robot->stopRobot();
                rc.robot->initializeJointVelocityInterface();
                rc.jvel_interface_running = true;
            } 
            else if (rc.jvel_interface_running && !rc.jvel_claimed) 
            {
                rc.robot->stopRobot();
                rc.jvel_interface_running = false;
            }

            // joint position
            if (!rc.jpos_interface_running && rc.jpos_claimed) 
            {
                rc.robot->stopRobot();
                rc.robot->initializeJointPositionInterface();
                rc.jpos_interface_running = true;
                rc.first_jpos_update = true;
            } 
            else if (rc.jpos_interface_running && !rc.jpos_claimed) 
            {
                rc.robot->stopRobot();
                rc.jpos_interface_running = false;
            }

            // cartesian velocity (+ optional elbow)
            if (!rc.cvel_interface_running && rc.cvel_claimed) 
            {
                std::fill(rc.cmd_cartesian_vel.begin(), rc.cmd_cartesian_vel.end(), 0.0);
                rc.robot->stopRobot();
                rc.robot->initializeCartesianVelocityInterface();
                if (!rc.elbow_interface_running && rc.elbow_claimed) 
                {
                    rc.elbow_interface_running = true;
                    rc.first_elbow_update = true;
                }
                rc.cvel_interface_running = true;
            } 
            else if (rc.cvel_interface_running && !rc.cvel_claimed) 
            {
                rc.robot->stopRobot();
                if (rc.elbow_interface_running) 
                {
                    rc.elbow_interface_running = false;
                    rc.elbow_claimed = false;
                }
                rc.cvel_interface_running = false;
            }

            // cartesian pose (+ optional elbow)
            if (!rc.cpose_interface_running && rc.cpose_claimed) 
            {
                rc.robot->stopRobot();
                rc.robot->initializeCartesianPoseInterface();
                if (!rc.elbow_interface_running && rc.elbow_claimed) 
                {
                    rc.elbow_interface_running = true;
                    rc.first_elbow_update = true;
                }
                rc.cpose_interface_running = true;
                rc.first_cpose_update = true;
            } 
            else if (rc.cpose_interface_running && !rc.cpose_claimed) 
            {
                rc.robot->stopRobot();
                if (rc.elbow_interface_running) 
                {
                    rc.elbow_interface_running = false;
                    rc.elbow_claimed = false;
                }
                rc.cpose_interface_running = false;
            }

            // final safety: elbow cannot run alone
            if (rc.elbow_claimed && !(rc.cvel_claimed || rc.cpose_claimed)) 
            {
                RCLCPP_FATAL(getLogger(), "Elbow cannot be commanded without cartesian velocity or pose interface");
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }



    rclcpp::Logger FrankaMultiHardwareInterface::getLogger() 
    {
        return rclcpp::get_logger("FrankaMultiHardwareInterface");
    }


    bool FrankaMultiHardwareInterface::findRobotByResourceName(const std::string& resource_name, size_t& robot_idx_out) const
    {
        size_t matched_robot_idx = static_cast<size_t>(-1);
        size_t match_count = 0;

        for (size_t candidate_robot_idx = 0; candidate_robot_idx < robots_.size(); ++candidate_robot_idx)
        {
            // const auto& robot_name_stem = robots_[candidate_robot_idx].name_stem;
            const auto& robot_name_stem = robots_[candidate_robot_idx].user_prefix;
            if (robot_name_stem.empty()) continue;

            if (resource_name.rfind(robot_name_stem, 0) == 0) // starts_with(robot_name_stem)
            {
                matched_robot_idx = candidate_robot_idx;
                ++match_count;
            }
        }

        if (match_count == 1) // is it unique?
        {
            robot_idx_out = matched_robot_idx;
            return true;
        }

        // no match or no unique match (ambiguous or legacy disabled)
        return false;
    }

    void FrankaMultiHardwareInterface::initializeCommandsFromState(RobotContext& rc) 
    {
        if (rc.first_elbow_update && rc.elbow_interface_running) 
        {
            rc.cmd_elbow = {rc.robot_state.elbow[0], rc.robot_state.elbow[1]};
            rc.first_elbow_update = false;
        }

        if (rc.first_jpos_update && rc.jpos_interface_running) 
        {
            rc.cmd_pos.assign(rc.robot_state.q.begin(), rc.robot_state.q.end());
            rc.first_jpos_update = false;
        }

        if (rc.first_cpose_update && rc.cpose_interface_running) 
        {
            rc.cmd_cartesian_pose.assign(rc.robot_state.O_T_EE.begin(), rc.robot_state.O_T_EE.end());
            rc.first_cpose_update = false;
        }
    }

    template <typename CommandType>
    bool hasInfinite(const CommandType& commands) 
    {
        return std::any_of(commands.begin(), commands.end(), [](double command) { return !std::isfinite(command); });
    }

    hardware_interface::return_type FrankaMultiHardwareInterface::writeOneRobot(RobotContext& rc) 
    {
        if (hasInfinite(rc.cmd_pos) || hasInfinite(rc.cmd_vel) || hasInfinite(rc.cmd_effort) || 
            hasInfinite(rc.cmd_cartesian_pose) || hasInfinite(rc.cmd_cartesian_vel) ||
            hasInfinite(rc.cmd_elbow)) 
        {
            return hardware_interface::return_type::ERROR;
        }

        if (rc.jvel_interface_running)
        {
            rc.robot->writeOnce(rc.cmd_vel);
        }
        else if (rc.effort_interface_running)
        {
            rc.robot->writeOnce(rc.cmd_effort);
        }
        else if (rc.jpos_interface_running && !rc.first_jpos_update)
        {
            rc.robot->writeOnce(rc.cmd_pos);
        }
        else if (rc.cvel_interface_running && rc.elbow_interface_running && !rc.first_elbow_update)
        {
            // Wait until the first read pass after robot controller is activated to write the elbow
            // command to the robot
            rc.robot->writeOnce(rc.cmd_cartesian_vel, rc.cmd_elbow);
        }
        else if (rc.cpose_interface_running && rc.elbow_interface_running && !rc.first_cpose_update && !rc.first_elbow_update) 
        {
            // Wait until the first read pass after robot controller is activated to write the elbow
            // command to the robot
            rc.robot->writeOnce(rc.cmd_cartesian_pose, rc.cmd_elbow);
        }
        else if (rc.cpose_interface_running && !rc.first_cpose_update)
        {
            // Wait until the first read pass after robot controller is activated to write the cartesian
            // pose
            rc.robot->writeOnce(rc.cmd_cartesian_pose);
        }
        else if (rc.cvel_interface_running && !rc.elbow_interface_running)
        {
            rc.robot->writeOnce(rc.cmd_cartesian_vel);
        }

        return hardware_interface::return_type::OK;
    }

}  // namespace franka_hardware

// Export plugin
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaMultiHardwareInterface,
                       hardware_interface::SystemInterface)