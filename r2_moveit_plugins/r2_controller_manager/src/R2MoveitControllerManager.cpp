#include <unistd.h> // gethostname

#include "r2_controller_manager/R2MoveitControllerManager.h"

#include <nasa_r2_common_msgs/Modes.h>
#include <nasa_r2_common_msgs/LabeledControllerJointSettings.h>
#include <nasa_r2_common_msgs/LabeledControllerPoseSettings.h>
#include <nasa_r2_common_msgs/LabeledJointState.h>
#include <nasa_r2_common_msgs/LabeledGains.h>
#include <r2_translation_msgs/R2Trajectory.h>

#include <pluginlib/class_list_macros.h>

#include <boost/math/constants/constants.hpp>

static const char* const R2_JOINT_TRAJECTORY_TOPIC  = "/r2_trajectory_translator";
static const char* const R2_TRAJECTORY_STATUS_TOPIC = "/goal_status";

static const char* const R2_MODES_TOPIC             = "/mode";
static const char* const R2_POSE_SETTINGS_TOPIC     = "/pose_ref_settings";
static const char* const R2_JOINT_SETTINGS_TOPIC    = "/joint_ref_settings";
static const char* const R2_TORQUE_SETTINGS_TOPIC   = "/desired_torque_limits";
static const char* const R2_JOINT_DYNAMICS_TOPIC    = "/desired_joint_dynamics";

#define PI boost::math::constants::pi<double>()

R2MoveItJointControllerHandle::R2MoveItJointControllerHandle(const std::string& name, const ControllerSettings& settings) :
    moveit_controller_manager::MoveItControllerHandle(name), node_handle_("~")
{
    // Trajectories that we receive to execute are forwarded to the R2 translator for ROS Fuerte
    trajectory_publisher_ = ros::Publisher(node_handle_.advertise<r2_translation_msgs::R2Trajectory>(R2_JOINT_TRAJECTORY_TOPIC, 1));
    // The status for currently executing trajectories are sent to us here
    actionLib_subscriber_ = ros::Subscriber(node_handle_.subscribe(R2_TRAJECTORY_STATUS_TOPIC, 10, &R2MoveItJointControllerHandle::trajectoryStatusCallback, this));

    char hostname[256];
    gethostname(hostname, 256);
    hostname_ = hostname;

    // These publishers are used to setup the robot
    ros::Publisher modes_publisher(node_handle_.advertise<nasa_r2_common_msgs::Modes>(R2_MODES_TOPIC, 1));
    ros::Publisher pose_settings_publisher(node_handle_.advertise<nasa_r2_common_msgs::LabeledControllerPoseSettings>(R2_POSE_SETTINGS_TOPIC, 1));
    ros::Publisher joint_torque_settings_publisher(node_handle_.advertise<nasa_r2_common_msgs::LabeledJointState>(R2_TORQUE_SETTINGS_TOPIC, 1));
    ros::Publisher joint_settings_publisher (node_handle_.advertise<nasa_r2_common_msgs::LabeledControllerJointSettings>(R2_JOINT_SETTINGS_TOPIC, 1));
    ros::Publisher joint_dynamics_settings_publisher(node_handle_.advertise<nasa_r2_common_msgs::LabeledGains>(R2_JOINT_DYNAMICS_TOPIC, 1));

    sleep(2);  // not sure why this is necessary, but I promise it is

    /*double TORAD = PI/180.0;
    double HZ_TORAD = PI * 2.0;

    // INITIALIZING SYSTEM
    // System mode - assign a Master
    nasa_r2_common_msgs::Modes modes_msg;
    modes_msg.header.stamp = ros::Time::now();
    modes_msg.originator = "user";
    modes_msg.master = hostname_;
    modes_publisher.publish(modes_msg);  // sending mode message first.


    // Controller pose settings
    nasa_r2_common_msgs::LabeledControllerPoseSettings controller_pose_msg;
    controller_pose_msg.originator = hostname_;
    controller_pose_msg.maxLinearVelocity = settings.max_linear_velocity;
    controller_pose_msg.maxLinearAcceleration = settings.max_linear_acceleration;
    controller_pose_msg.maxRotationalVelocity = settings.max_rotational_velocity;
    controller_pose_msg.maxRotationalAcceleration = settings.max_rotational_acceleration;
    pose_settings_publisher.publish(controller_pose_msg);

    // Torso joint desired dynamics - stiffness
    nasa_r2_common_msgs::LabeledGains joint_dynamics_message;
    joint_dynamics_message.originator = hostname_;
    joint_dynamics_message.joint_names = settings.all_joint_names;

    for(size_t i = 0; i < settings.all_joint_names.size(); ++i)
    {
        const R2ControllerJointSettings& joint_settings = settings.joint_settings.find(settings.all_joint_names[i])->second;
        joint_dynamics_message.naturalFreq.push_back(joint_settings.natural_frequency);
        joint_dynamics_message.dampingRatio.push_back(joint_settings.damping_ratio);
        joint_dynamics_message.windupLimit.push_back(joint_settings.windup_limit);
    }
    joint_dynamics_settings_publisher.publish(joint_dynamics_message);

    // Joint torque limits
    nasa_r2_common_msgs::LabeledJointState joint_torque_limits;
    joint_torque_limits.originator = hostname_;
    joint_torque_limits.name = settings.all_joint_names;

    for(size_t i = 0; i < joint_torque_limits.name.size(); ++i)
    {
        const R2ControllerJointSettings& joint_settings = settings.joint_settings.find(joint_torque_limits.name[i])->second;
        joint_torque_limits.effort.push_back(joint_settings.max_torque);
    }

    joint_torque_settings_publisher.publish(joint_torque_limits);

    // Controller joint settings
    nasa_r2_common_msgs::LabeledControllerJointSettings controller_joint_msg;
    controller_joint_msg.originator = hostname_;
    controller_joint_msg.joint_names = settings.all_joint_names;

    for(size_t i = 0; i < controller_joint_msg.joint_names.size(); ++i)
    {
        const R2ControllerJointSettings& joint_settings = settings.joint_settings.find(controller_joint_msg.joint_names[i])->second;
        controller_joint_msg.jointVelocityLimits.push_back(joint_settings.max_velocity); // rad/sec
        controller_joint_msg.jointAccelerationLimits.push_back(joint_settings.max_acceleration); // rad/sec^2
    }
    joint_settings_publisher.publish(controller_joint_msg);
    sleep(1);*/
}

R2MoveItJointControllerHandle::~R2MoveItJointControllerHandle()
{
}

bool R2MoveItJointControllerHandle::sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
    if (trajectory.multi_dof_joint_trajectory.points.size() > 0)
        ROS_WARN("R2MoveItJointController cannot execute MultiDOF trajectories");

    trajectory_msgs::JointTrajectory trajectory_msg(trajectory.joint_trajectory);

    last_trajectory_id_ = "joint_trajectory";
    if (trajectory_publisher_.getNumSubscribers() < 1)
    {
        ROS_ERROR("There are no subscribers to %s.  NOT executing trajectory %s", R2_JOINT_TRAJECTORY_TOPIC, last_trajectory_id_.c_str());
        return false;
    }

    for (size_t i = 0; i < trajectory_msg.points.size(); ++i)
    {
        unsigned int numPoints = trajectory_msg.points[i].positions.size();

        trajectory_msg.points[i].velocities.assign(numPoints, 0);
        trajectory_msg.points[i].accelerations.assign(numPoints, 0);

        trajectory_msg.points[i].time_from_start.sec = -1;
        trajectory_msg.points[i].time_from_start.nsec = 0;
    }

    // Copy the message into the custom R2 trajectory message to be translated
    r2_translation_msgs::R2Trajectory r2_trajectory_msg;
    r2_trajectory_msg.frame_id = last_trajectory_id_;
    r2_trajectory_msg.originator = hostname_;
    r2_trajectory_msg.joint_names = trajectory_msg.joint_names;
    for (size_t i = 0; i < trajectory_msg.points.size(); ++i)
    {
        r2_translation_msgs::R2TrajectoryPoint point;
        point.positions = trajectory_msg.points[i].positions;
        point.velocities = trajectory_msg.points[i].velocities;
        point.accelerations = trajectory_msg.points[i].accelerations;
        point.time_from_start = trajectory_msg.points[i].time_from_start;

        r2_trajectory_msg.points.push_back(point);
    }

    trajectory_status_[last_trajectory_id_] = actionlib_msgs::GoalStatus::PENDING; // Set unknown status for trajectory
    trajectory_publisher_.publish(r2_trajectory_msg);
    return true;
}

bool R2MoveItJointControllerHandle::cancelExecution()
{
    ROS_WARN("R2MoveItJointControllerHandle cannot cancel execution");
    return false;
}

bool R2MoveItJointControllerHandle::waitForExecution(const ros::Duration& timeout)
{
    if (actionLib_subscriber_.getNumPublishers() < 1)
    {
        ROS_WARN("No publishers on %s.  Not waiting for trajectory execution.  Assuming success.", R2_TRAJECTORY_STATUS_TOPIC);
        trajectory_status_["joint_trajectory"] = actionlib_msgs::GoalStatus::SUCCEEDED;
        return true;
    }

    if (last_trajectory_id_ == "")
    {
        ROS_WARN("No trajectory being executed");
        return true;
    }

    unsigned int rate = 50; // Rate at which we check, in Hz
    unsigned int delay = (1.0/rate)*1000000; // microseconds

    ros::Time start = ros::Time::now();
    bool timeout_reached = false;
    do
    {
        //ros::spinOnce(); // Process events during wait, just in case
        usleep(delay);

        if ((ros::Time::now() - start) > timeout)
            timeout_reached = true;
        //else
        //    ROS_WARN("%f secs until timeout", (timeout - (ros::Time::now() - start)).toSec());

    } while (trajectory_status_[last_trajectory_id_] <= actionlib_msgs::GoalStatus::ACTIVE && !timeout_reached);

    return timeout_reached;
}

moveit_controller_manager::ExecutionStatus R2MoveItJointControllerHandle::getLastExecutionStatus()
{
    if (last_trajectory_id_ == "")
        return moveit_controller_manager::ExecutionStatus::UNKNOWN;

    switch(trajectory_status_[last_trajectory_id_])
    {
        case actionlib_msgs::GoalStatus::ACTIVE:
            return moveit_controller_manager::ExecutionStatus::RUNNING;

        case actionlib_msgs::GoalStatus::PREEMPTED:
            return moveit_controller_manager::ExecutionStatus::PREEMPTED;

        case actionlib_msgs::GoalStatus::SUCCEEDED:
            return moveit_controller_manager::ExecutionStatus::SUCCEEDED;

        case actionlib_msgs::GoalStatus::ABORTED:
            return moveit_controller_manager::ExecutionStatus::ABORTED;

        case actionlib_msgs::GoalStatus::REJECTED:
            return moveit_controller_manager::ExecutionStatus::FAILED;

        default:
            return moveit_controller_manager::ExecutionStatus::UNKNOWN;
    }
}

// Callback that updates the trajectory_status_ map
void R2MoveItJointControllerHandle::trajectoryStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
    for (size_t i = 0; i < status->status_list.size(); ++i)
    {
        const std::string& trajectoryID = status->status_list[i].goal_id.id;
        trajectory_status_[trajectoryID] = (unsigned int)status->status_list[i].status;
    }
}

////////////////////////////////////////////////////////////////////////////////

R2MoveItControllerManager::R2MoveItControllerManager() : node_handle_("~")
{
    double TORAD = PI/180.0;
    double HZ_TORAD = PI * 2.0;

    // Reading in controller parameters
    XmlRpc::XmlRpcValue controller_list;
    if (node_handle_.hasParam("controller_list"))
    {
        node_handle_.getParam("controller_list", controller_list);
        if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_WARN("Controller list should be specified as an array");
        else
        {
            for (int i = 0 ; i < controller_list.size() ; ++i)
            {
                if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
                    ROS_WARN("Name and joints must be specifed for each controller");
                else
                {
                    try
                    {
                        ControllerInformation ci;
                        ci.active_ = false;
                        std::string name = std::string(controller_list[i]["name"]);
                        if (controller_list[i].hasMember("default"))
                        {
                            try
                            {
                                if (controller_list[i]["default"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                                {
                                    ci.default_ = controller_list[i]["default"];
                                }
                                else
                                {
                                    std::string def = std::string(controller_list[i]["default"]);
                                    std::transform(def.begin(), def.end(), def.begin(), ::tolower);
                                    if (def == "true" || def == "yes")
                                        ci.default_ = true;
                                }
                            }
                            catch (...)
                            {
                            }
                        }

                        if (controller_list[i]["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray)
                        {
                            int nj = controller_list[i]["joints"].size();
                            for (int j = 0 ; j < nj ; ++j)
                                ci.joints_.push_back(std::string(controller_list[i]["joints"][j]));
                        }
                        else
                            ROS_WARN_STREAM("The list of joints for controller " << name << " is not specified as an array");

                        // Parsing controller settings
                        if (controller_list[i].hasMember("pose_settings"))
                        {
                            R2ControllerJointSettings default_joint_settings;
                            ControllerSettings controller_settings;

                            int nj = controller_list[i]["all_joints"].size();
                            for (int j = 0 ; j < nj ; ++j)
                                controller_settings.all_joint_names.push_back(std::string(controller_list[i]["all_joints"][j]));

                            bool radians = true;
                            if (controller_list[i]["pose_settings"].hasMember("radians"))
                                radians = controller_list[i]["pose_settings"]["radians"];
                            else
                                ROS_WARN("%s: pose_settings do not specify units.  Assuming radians.", name.c_str());

                            if (controller_list[i]["pose_settings"].hasMember("max"))
                                radians = controller_list[i]["pose_settings"]["radians"];

                            controller_settings.max_linear_velocity = controller_list[i]["pose_settings"]["max_linear_velocity"];
                            controller_settings.max_linear_acceleration = controller_list[i]["pose_settings"]["max_linear_acceleration"];
                            controller_settings.max_rotational_velocity = controller_list[i]["pose_settings"]["max_rotational_velocity"];
                            if (!radians) controller_settings.max_rotational_velocity *= TORAD;
                            controller_settings.max_rotational_acceleration = controller_list[i]["pose_settings"]["max_rotational_acceleration"];
                            if (!radians) controller_settings.max_rotational_acceleration *= TORAD;

                            // Parsing joint_stiffness
                            if (!controller_list[i].hasMember("joint_stiffness"))
                            {
                                ROS_ERROR("No joint stiffness values specified in configuration for %s", name.c_str());
                                return;
                            }

                            // Default joint stiffness
                            if (controller_list[i]["joint_stiffness"].hasMember("default"))
                            {
                                if (controller_list[i]["joint_stiffness"]["default"].hasMember("radians"))
                                    radians = controller_list[i]["joint_stiffness"]["default"]["radians"];
                                else
                                {
                                    ROS_WARN("%s: default joint_stiffness values do not specify units.  Assuming radians.", name.c_str());
                                    radians = true;
                                }

                                default_joint_settings.natural_frequency = controller_list[i]["joint_stiffness"]["default"]["natural_frequency"];
                                if (!radians) default_joint_settings.natural_frequency *= HZ_TORAD;
                                default_joint_settings.damping_ratio = controller_list[i]["joint_stiffness"]["default"]["damping_ratio"];
                                default_joint_settings.windup_limit = controller_list[i]["joint_stiffness"]["default"]["windup_limit"];

                                controller_settings.default_joint_settings = default_joint_settings;
                            }
                            else
                            {
                                ROS_ERROR("No default joint stiffness settings specified for  %s", name.c_str());
                                return;
                            }

                            // Parsing default joint torque
                            if (!controller_list[i].hasMember("joint_torque"))
                            {
                                ROS_ERROR("No joint torque values specified in configuration for %s", name.c_str());
                                return;
                            }

                            if (controller_list[i]["joint_torque"].hasMember("default"))
                            {
                                default_joint_settings.max_torque = controller_list[i]["joint_torque"]["default"]["max_torque"];
                            }
                            else
                            {
                                ROS_ERROR("No default joint torque values specified in configuration for %s", name.c_str());
                                return;
                            }

                            // Parsing default joint speed
                            if (!controller_list[i].hasMember("joint_speed"))
                            {
                                ROS_ERROR("No joint speed values specified in configuration for %s", name.c_str());
                                return;
                            }

                            if (controller_list[i]["joint_speed"].hasMember("default"))
                            {
                                if (controller_list[i]["joint_speed"]["default"].hasMember("radians"))
                                    radians = controller_list[i]["joint_speed"]["default"]["radians"];
                                else
                                {
                                    ROS_WARN("%s: default joint_speed values do not specify units.  Assuming radians.", name.c_str());
                                    radians = true;
                                }

                                default_joint_settings.max_velocity = controller_list[i]["joint_speed"]["default"]["velocity_limit"];
                                default_joint_settings.max_acceleration = controller_list[i]["joint_speed"]["default"]["acceleration_limit"];
                                if (!radians)
                                {
                                    default_joint_settings.max_velocity *= TORAD;
                                    default_joint_settings.max_acceleration *= TORAD;
                                }
                            }
                            else
                            {
                                ROS_ERROR("No default joint torque values specified in configuration for %s", name.c_str());
                                return;
                            }

                            // Generate a joint settings object for each joint
                            for(size_t j = 0; j < controller_settings.all_joint_names.size(); ++j)
                                controller_settings.joint_settings[controller_settings.all_joint_names[j]] = default_joint_settings;


                            // Individual Joint stiffness
                            if (controller_list[i]["joint_stiffness"].hasMember("joints"))
                            {
                                if (controller_list[i]["joint_stiffness"]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
                                    ROS_WARN("%s: joint_stiffness joints should be specified as an array", name.c_str());

                                for (int j = 0 ; j < controller_list[i]["joint_stiffness"]["joints"].size() ; ++j)
                                {
                                    std::string joint_name = std::string(controller_list[i]["joint_stiffness"]["joints"][j]["name"]);
                                    R2ControllerJointSettings& joint_settings = controller_settings.joint_settings[joint_name];

                                    if (controller_list[i]["joint_stiffness"]["joints"][j].hasMember("radians"))
                                        radians = controller_list[i]["joint_stiffness"]["joints"][j]["radians"];
                                    else
                                    {
                                        ROS_WARN("%s: joint_stiffness values for %s do not specify units.  Assuming radians.", name.c_str(), joint_name.c_str());
                                        radians = true;
                                    }

                                    joint_settings.natural_frequency = controller_list[i]["joint_stiffness"]["joints"][j]["natural_frequency"];
                                    if (!radians) joint_settings.natural_frequency *= HZ_TORAD;
                                    joint_settings.damping_ratio = controller_list[i]["joint_stiffness"]["joints"][j]["damping_ratio"];
                                    joint_settings.windup_limit = controller_list[i]["joint_stiffness"]["joints"][j]["windup_limit"];
                                }
                            }

                            // Individual joint torques
                            if (controller_list[i]["joint_torque"].hasMember("joints"))
                            {
                                if (controller_list[i]["joint_torque"]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
                                    ROS_WARN("%s: joint_torque joints should be specified as an array", name.c_str());

                                for (int j = 0 ; j < controller_list[i]["joint_torque"]["joints"].size() ; ++j)
                                {
                                    std::string joint_name = std::string(controller_list[i]["joint_torque"]["joints"][j]["name"]);
                                    R2ControllerJointSettings& joint_settings = controller_settings.joint_settings[joint_name];

                                    if (controller_list[i]["joint_torque"]["joints"][j].hasMember("radians"))
                                        radians = controller_list[i]["joint_torque"]["joints"][j]["radians"];
                                    else
                                    {
                                        ROS_WARN("%s: joint_torque values for %s do not specify units.  Assuming radians.", name.c_str(), joint_name.c_str());
                                        radians = true;
                                    }

                                    joint_settings.max_torque = controller_list[i]["joint_torque"]["joints"][j]["max_torque"];
                                }
                            }

                            // Individual joint speeds
                            if (controller_list[i]["joint_speed"].hasMember("joints"))
                            {
                                if (controller_list[i]["joint_speed"]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
                                    ROS_WARN("%s: joint_speed joints should be specified as an array", name.c_str());

                                for (int j = 0 ; j < controller_list[i]["joint_speed"]["joints"].size() ; ++j)
                                {
                                    std::string joint_name = std::string(controller_list[i]["joint_speed"]["joints"][j]["name"]);
                                    R2ControllerJointSettings& joint_settings = controller_settings.joint_settings[joint_name];

                                    if (controller_list[i]["joint_speed"]["joints"][j].hasMember("radians"))
                                        radians = controller_list[i]["joint_speed"]["joints"][j]["radians"];
                                    else
                                    {
                                        ROS_WARN("%s: joint_speed values for %s do not specify units.  Assuming radians.", name.c_str(), joint_name.c_str());
                                        radians = true;
                                    }

                                    joint_settings.max_velocity = controller_list[i]["joint_speed"]["joints"][j]["velocity_limit"];
                                    joint_settings.max_acceleration = controller_list[i]["joint_speed"]["joints"][j]["acceleration_limit"];
                                    if (!radians)
                                    {
                                        joint_settings.max_velocity *= TORAD;
                                        joint_settings.max_acceleration *= TORAD;
                                    }
                                }
                            }

                            ci.settings_ = controller_settings;
                            if (!ci.joints_.empty())
                                controllers_[name] = ci;
                        }
                        else
                        {
                            ROS_WARN("%s: No pose_settings found", name.c_str());
                        }
                    }
                    catch (...)
                    {
                      ROS_ERROR("Unable to parse controller information");
                    }
                }
            }
        }
    }

    else
    {
        ROS_ERROR("No controller_list found.  Not using R2 controller manager");
    }

    if (controllers_.size() == 1)
        controllers_.begin()->second.active_ = true;
}

R2MoveItControllerManager::~R2MoveItControllerManager()
{
}

// Allocate a new controller handle with the given name
moveit_controller_manager::MoveItControllerHandlePtr R2MoveItControllerManager::getControllerHandle(const std::string& name)
{
    moveit_controller_manager::MoveItControllerHandlePtr new_handle;
    if (name == "r2_legs_controller" || name == "r2_arms_controller")
        new_handle.reset(new R2MoveItJointControllerHandle(name, controllers_[name].settings_));

    return new_handle;
}

void R2MoveItControllerManager::getControllersList(std::vector<std::string> &names)
{
    std::map<std::string, ControllerInformation>::const_iterator it;
    for(it = controllers_.begin(); it != controllers_.end(); ++it)
        names.push_back(it->first);
}

void R2MoveItControllerManager::getActiveControllers(std::vector<std::string> &names)
{
    std::map<std::string, ControllerInformation>::const_iterator it;
    for(it = controllers_.begin(); it != controllers_.end(); ++it)
        if (it->second.active_)
            names.push_back(it->first);
}

void R2MoveItControllerManager::getControllerJoints(const std::string &name, std::vector<std::string> &joints)
{
    std::map<std::string, ControllerInformation>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      joints = it->second.joints_;
}

moveit_controller_manager::MoveItControllerManager::ControllerState
R2MoveItControllerManager::getControllerState(const std::string &name)
{
    moveit_controller_manager::MoveItControllerManager::ControllerState state;

    std::map<std::string, ControllerInformation>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
        state.active_ = it->second.active_;
        state.default_ = it->second.default_;
    }

    return state;
}

// This doesn't really do much
bool R2MoveItControllerManager::switchControllers(const std::vector<std::string> &activate,
                                                  const std::vector<std::string> &deactivate)
{
    for(size_t i = 0; i < activate.size(); ++i)
    {
        std::map<std::string, ControllerInformation>::iterator it = controllers_.find(activate[i]);
        if (it != controllers_.end())
            it->second.active_ = true;
    }

    for(size_t i = 0; i < deactivate.size(); ++i)
    {
        std::map<std::string, ControllerInformation>::iterator it = controllers_.find(deactivate[i]);
        if (it != controllers_.end())
            it->second.active_ = false;
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(R2MoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager)