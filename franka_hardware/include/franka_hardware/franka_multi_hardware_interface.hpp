#pragma once

#include <array>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <franka/robot_state.h>

#include "franka_action_server.hpp"
#include "franka_hardware/franka_executor.hpp"
#include "franka_hardware/franka_param_service_server.hpp"
#include "franka_hardware/robot.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hardware 
{
    class FrankaMultiHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            FrankaMultiHardwareInterface();
            FrankaMultiHardwareInterface(const FrankaMultiHardwareInterface&) = delete;
            FrankaMultiHardwareInterface& operator=(const FrankaMultiHardwareInterface&) = delete;
            FrankaMultiHardwareInterface(FrankaMultiHardwareInterface&&) = delete;
            FrankaMultiHardwareInterface& operator=(FrankaMultiHardwareInterface&&) = delete;
            ~FrankaMultiHardwareInterface() override;  // needs to join worker threads

            CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
            hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;


            static constexpr size_t kNumberOfJoints = 7;

        private:
            struct RobotContext 
            {
                std::string arm_id;   // panda, fr3, ...
                unsigned int arm_idx;
                std::string robot_ip;
                std::string user_prefix;   // optional: "left", "right", "0_fr3", ...
                std::string name_stem;     // computed: "left_fr3" or "fr3"


                std::shared_ptr<Robot> robot;
                std::shared_ptr<FrankaParamServiceServer> service_node;
                std::shared_ptr<ActionServer> action_node;
                std::shared_ptr<FrankaExecutor> executor;

                // command buffers (joint)
                std::vector<double> cmd_effort{0, 0, 0, 0, 0, 0, 0};
                std::vector<double> cmd_pos{0, 0, 0, 0, 0, 0, 0};
                std::vector<double> cmd_vel{0, 0, 0, 0, 0, 0, 0};

                // command buffers (cartesian/elbow)
                std::vector<double> cmd_cartesian_vel{0, 0, 0, 0, 0, 0};  // vx vy vz wx wy wz
                std::vector<double> cmd_cartesian_pose{1, 0, 0, 0,
                                                       0, 1, 0, 0,
                                                       0, 0, 1, 0,
                                                       0, 0, 0, 1};      // 4x4 column-major
                std::vector<double> cmd_elbow{0, 0};                     // joint_3_position, joint_4_sign

                // state buffers
                std::array<double, kNumberOfJoints> q{0, 0, 0, 0, 0, 0, 0};
                std::array<double, kNumberOfJoints> dq{0, 0, 0, 0, 0, 0, 0};
                std::array<double, kNumberOfJoints> tau{0, 0, 0, 0, 0, 0, 0};

                std::array<double, 16> O_T_EE{1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 1, 0,
                                              0, 0, 0, 1};
                std::array<double, 2> elbow{0, 0};

                // libfranka handles exposed via state interface (pointer punning like single version)
                franka::RobotState robot_state{};
                franka::RobotState* robot_state_ptr{&robot_state};
                Model* model_ptr{nullptr};
                double robot_time_state{0.0};
                bool comm_constraints_recovery_sent{false};

                // mode flags
                bool jvel_claimed{false},   jvel_interface_running{false};
                bool jpos_claimed{false},   jpos_interface_running{false};
                bool effort_claimed{false}, effort_interface_running{false};

                bool cpose_claimed{false},  cpose_interface_running{false};
                bool cvel_claimed{false},   cvel_interface_running{false};
                bool elbow_claimed{false},  elbow_interface_running{false};

                // first-update guards (prevents jumps)
                bool first_jpos_update{true};
                bool first_cpose_update{true};
                bool first_elbow_update{true};

                // ---- parallel worker thread synchronization ----
                // std::mutex/condition_variable are non-movable, so wrap them in a
                // heap-allocated struct. unique_ptr IS movable -> RobotContext is movable.
                //
                // Per-cycle protocol (one exchange per 1 kHz tick):
                //   Phase A  main -> worker : provide cmd_* buffers, signal CYCLE_BEGIN
                //   Phase B  worker         : readOnce() -> fill pending_state
                //                             writeOnce() with the cmd buffers provided
                //   Phase C  worker -> main : signal CYCLE_DONE
                //   Phase D  main           : commit pending_state -> robot_state
                //
                // READ and WRITE are merged into a single worker wake-up so the total
                // round-trip is one mutex lock/unlock pair instead of two, keeping the
                // hot path well under 1 ms even on a loaded system.
                enum class WorkerCmd { IDLE, CYCLE, SHUTDOWN };

                struct WorkerSync
                {
                    // main->worker: "start a cycle"
                    std::mutex              cmd_mtx;
                    std::condition_variable cmd_cv;
                    WorkerCmd               cmd{WorkerCmd::IDLE};

                    // worker->main: "cycle done"
                    std::mutex              done_mtx;
                    std::condition_variable done_cv;
                    bool                    done{false};

                    // State read by worker, committed by main after all workers finish
                    franka::RobotState      pending_state{};

                    // write_only: set to ERROR if writeOneRobot fails
                    hardware_interface::return_type cycle_result{hardware_interface::return_type::OK};

                    // Exception transport
                    std::exception_ptr      worker_exception{nullptr};
                };

                std::unique_ptr<WorkerSync> sync{std::make_unique<WorkerSync>()};
            };  // struct RobotContext

            struct InterfaceInfoMulti 
            {
                std::string interface_type;
                size_t size;

                // Set/get claimed flag on RobotContext for this interface type
                std::function<void(RobotContext&, bool)> set_claimed;
                std::function<bool(const RobotContext&)> get_claimed;  // optional (not required in prepare)
            };

            


            struct JointIndex 
            {
                size_t robot{0};  // 0(left), 1(right) 
                size_t joint{0};  // 0(joint1), ..., 6(joint7)
            };

            // ---------- constants / interface type names ----------
            const std::string k_hw_if_cartesian_velocity_{"cartesian_velocity"};
            const std::string k_hw_if_cartesian_pose_command_{"cartesian_pose_command"};
            const std::string k_hw_if_elbow_command_{"elbow_command"};

            const std::string k_hw_if_cartesian_pose_state_{"cartesian_pose_state"};
            const std::string k_hw_if_elbow_state_{"elbow_state"};

            const std::string k_robot_state_interface_name_{"robot_state"};
            const std::string k_robot_model_interface_name_{"robot_model"};
            const int kSupportedControlInterfaceMajor_{0};

            const std::vector<InterfaceInfoMulti> command_interfaces_info_multi_;

            // ---------- storage ----------
            std::vector<RobotContext> robots_;

            // joint name -> (robot_idx, local_joint_idx)
            std::unordered_map<std::string, JointIndex> joint_map_;

            // exported command interfaces "name/type" (for strict checking like single code)
            std::unordered_set<std::string> exported_command_interfaces_;

            // For overly-broad start list detection (scaled by number of robots)
            const size_t max_number_start_interfaces_per_robot_{45};

            // ---------- parallel worker threads (one per robot, persistent) ----------
            std::vector<std::thread> worker_threads_;
            bool last_cycle_ok_{true};  // result of last dispatchCycleAndWait(), read by write()

            // ---------- helpers ----------
            static rclcpp::Logger getLogger();

            // find robot by resource name using prefix match
            bool findRobotByResourceName(const std::string& resource_name, size_t& robot_idx_out) const;

            // Initialize command buffers on first read after a mode is running
            void initializeCommandsFromState(RobotContext& rc);

            hardware_interface::return_type writeOneRobot(RobotContext& rc);

            // Start/stop persistent worker threads
            void startWorkerThreads();
            void stopWorkerThreads();

            // Trigger one full READ+WRITE cycle on all worker threads in parallel and
            // block until every worker signals completion.
            // Returns false if any worker threw an exception (already logged).
            bool dispatchCycleAndWait();
    };

}  // namespace franka_hardware