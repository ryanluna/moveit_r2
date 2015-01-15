#ifndef R2_CONTROLLER_MANAGER_R2_CONTROLLER_MANAGER
#define R2_CONTROLLER_MANAGER_R2_CONTROLLER_MANAGER

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <map>
#include <string>

struct R2ControllerJointSettings
{
    double natural_frequency;
    double damping_ratio;
    double windup_limit;

    double max_torque;
    double max_velocity;
    double max_acceleration;
};

struct ControllerSettings
{
    double max_linear_velocity;
    double max_linear_acceleration;
    double max_rotational_velocity;
    double max_rotational_acceleration;

    R2ControllerJointSettings default_joint_settings;
    std::map<std::string, R2ControllerJointSettings> joint_settings;
    std::vector<std::string> all_joint_names;
};

/// An instance of a joint controller interface for R2 in MoveIt
class R2MoveItJointControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
    R2MoveItJointControllerHandle(const std::string& name, const ControllerSettings& settings);
    ~R2MoveItJointControllerHandle();

    virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory);
    virtual bool cancelExecution();

    virtual bool waitForExecution(const ros::Duration& timeout = ros::Duration(0));
    virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus();

protected:
    // Initialize the robot for executing trajectories from this machine
    void initialize(const std::string& master);

    // Callback for trajectory status updates
    void trajectoryStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status);

    // Send trajectories to execute here
    ros::Publisher trajectory_publisher_;
    // Receive updates on trajectories being executed here
    ros::Subscriber actionLib_subscriber_;

    // The status of trajectories
    std::map<std::string, unsigned int> trajectory_status_;

    std::string hostname_;
    std::string last_trajectory_id_;
    ros::NodeHandle node_handle_;
};

/// The controller manager for R2 using MoveIt
class R2MoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
    R2MoveItControllerManager();
    ~R2MoveItControllerManager();

    virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name);

    /** \brief Controllers are managed by name. Given a name, return an object that can
               perform operations on the corresponding controller. */
    virtual void getControllersList(std::vector<std::string> &names);

    /** \brief Get the list of active controllers. If there is only one controller
               in the system, this will be active. In cases where multiple
               controllers exist, and they operate on overlaping sets of joints,
               not all controllers should be sctive at the same time. */
    virtual void getActiveControllers(std::vector<std::string> &names);

    /** \brief In order to decide which controller to use, it is necessary to
               reason about the joints a controller operates on. This function
               reports the joints a controller operates on, given the
               controller name. */
    virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints);

    /** \brief Report the state of a controller, given its name. */
    virtual ControllerState getControllerState(const std::string &name);

    /** \brief Activate and deactivate controllers */
    virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate);

protected:
    ros::NodeHandle node_handle_;

    struct ControllerInformation
    {
        ControllerInformation() : default_(false), active_(false) {}

        bool default_;
        bool active_;
        std::vector<std::string> joints_;
        ControllerSettings settings_;
    };

    std::map<std::string, ControllerInformation> controllers_;
};

#endif