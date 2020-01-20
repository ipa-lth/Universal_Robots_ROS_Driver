[1mdiff --git a/ur_controllers/include/ur_controllers/hardware_interface_adapter.h b/ur_controllers/include/ur_controllers/hardware_interface_adapter.h[m
[1mindex 116f38c..e52ccd5 100644[m
[1m--- a/ur_controllers/include/ur_controllers/hardware_interface_adapter.h[m
[1m+++ b/ur_controllers/include/ur_controllers/hardware_interface_adapter.h[m
[36m@@ -115,7 +115,9 @@[m [mtemplate <class State>[m
 class ClosedLoopHardwareInterfaceAdapter[m
 {[m
 public:[m
[31m-  ClosedLoopHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}[m
[32m+[m[32m  ClosedLoopHardwareInterfaceAdapter() : joint_handles_ptr_(0)[m
[32m+[m[32m  {[m
[32m+[m[32m  }[m
 [m
   bool init(std::vector<ur_controllers::ScaledJointHandle>& joint_handles, ros::NodeHandle& controller_nh)[m
   {[m
[36m@@ -150,7 +152,10 @@[m [mpublic:[m
 [m
   void starting(const ros::Time& /*time*/)[m
   {[m
[31m-    if (!joint_handles_ptr_) {return;}[m
[32m+[m[32m    if (!joint_handles_ptr_)[m
[32m+[m[32m    {[m
[32m+[m[32m      return;[m
[32m+[m[32m    }[m
 [m
     // Reset PIDs, zero commands[m
     for (unsigned int i = 0; i < pids_.size(); ++i)[m
[36m@@ -160,12 +165,12 @@[m [mpublic:[m
     }[m
   }[m
 [m
[31m-  void stopping(const ros::Time& /*time*/) {}[m
[32m+[m[32m  void stopping(const ros::Time& /*time*/)[m
[32m+[m[32m  {[m
[32m+[m[32m  }[m
 [m
[31m-  void updateCommand(const ros::Time&     /*time*/,[m
[31m-                     const ros::Duration& period,[m
[31m-                     const State&         desired_state,[m
[31m-                     const State&         state_error)[m
[32m+[m[32m  void updateCommand(const ros::Time& /*time*/, const ros::Duration& period, const State& desired_state,[m
[32m+[m[32m                     const State& state_error)[m
   {[m
     const unsigned int n_joints = joint_handles_ptr_->size();[m
 [m
[36m@@ -178,7 +183,8 @@[m [mpublic:[m
     // Update PIDs[m
     for (unsigned int i = 0; i < n_joints; ++i)[m
     {[m
[31m-      const double command = (desired_state.velocity[i] * velocity_ff_[i]) + pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);[m
[32m+[m[32m      const double command = (desired_state.velocity[i] * velocity_ff_[i]) +[m
[32m+[m[32m                             pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);[m
       (*joint_handles_ptr_)[i].setCommand(command);[m
     }[m
   }[m
[36m@@ -191,18 +197,14 @@[m [mprivate:[m
 [m
   std::vector<ur_controllers::ScaledJointHandle>* joint_handles_ptr_;[m
 };[m
[31m-}[m
[32m+[m[32m}  // namespace ur_controllers[m
 [m
 /**[m
  * \brief Adapter for an velocity-controlled hardware interface. Maps position and velocity errors to velocity commands[m
  * through a velocity PID loop.[m
  *[m
[31m- * The following is an example configuration of a controller that uses this adapter. Notice the \p gains and \p velocity_ff[m
[31m- * entries:[m
[31m- * \code[m
[31m- * head_controller:[m
[31m- *   type: "velocity_controllers/ScaledJointTrajectoryController"[m
[31m- *   joints:[m
[32m+[m[32m * The following is an example configuration of a controller that uses this adapter. Notice the \p gains and \p[m
[32m+[m[32m * velocity_ff entries: \code head_controller: type: "velocity_controllers/ScaledJointTrajectoryController" joints:[m
  *     - head_1_joint[m
  *     - head_2_joint[m
  *   gains:[m
[36m@@ -221,7 +223,9 @@[m [mprivate:[m
  * \endcode[m
  */[m
 template <class State>[m
[31m-class HardwareInterfaceAdapter<ur_controllers::ScaledVelocityJointInterface, State> : public ur_controllers::ClosedLoopHardwareInterfaceAdapter<State>[m
[31m-{};[m
[32m+[m[32mclass HardwareInterfaceAdapter<ur_controllers::ScaledVelocityJointInterface, State>[m
[32m+[m[32m  : public ur_controllers::ClosedLoopHardwareInterfaceAdapter<State>[m
[32m+[m[32m{[m
[32m+[m[32m};[m
 [m
 #endif  // ifndef UR_CONTROLLERS_HARDWARE_INTERFACE_ADAPTER_H_INCLUDED[m
