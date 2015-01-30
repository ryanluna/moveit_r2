/* Author: Ryan Luna */

#include "moveit_r2_kinematics/moveit_r2_tree_kinematics.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <moveit/rdf_loader/rdf_loader.h>

// Conversion from KDL pose to ROS msg
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>

#include <eigen_conversions/eigen_msg.h>

// For plugin library
#include <class_loader/class_loader.h>

//register MoveItR2TreeKinematicsPlugin as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_r2_kinematics
{

//// TreeIkRequest ////
void TreeIkRequest::addFixedLink(const std::string& link_name)                     { fixed_links_.push_back(link_name); }
void TreeIkRequest::setJointValues(const std::vector<double>& values)              { initial_joints_ = values; }
const std::vector<std::string>& TreeIkRequest::getFixedLinks()               const { return fixed_links_; }
const std::vector<std::string>& TreeIkRequest::getMovingLinks()              const { return moving_links_; }
const std::vector<geometry_msgs::Pose>& TreeIkRequest::getMovingLinkPoses()  const { return poses_; }
const std::vector<double>& TreeIkRequest::getJointValues()                   const { return initial_joints_; }
const geometry_msgs::Pose& TreeIkRequest::getWorldState()                    const { return world_state_; }
const std::vector<double>& TreeIkRequest::getWorldStateRPY()                 const { return world_state_rpy_; }
const std::vector<KdlTreeIk::NodePriority>& TreeIkRequest::getPriorities()   const { return priorities_; }

void TreeIkRequest::addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const int priority)
{
    moving_links_.push_back(link_name);
    poses_.push_back(pose);
    KdlTreeIk::NodePriority priority_vec;
    for (size_t i = 0; i < 6; ++i) // Set each DOF to the given priority
        priority_vec[i] = priority;
    priorities_.push_back(priority_vec);
}

void TreeIkRequest::addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const std::vector<int>& priority)
{
    if (priority.size() != 6)
    {
        ROS_ERROR("Expected priority vector of size 6, but received vector with size %lu.  Not adding link pose", priority.size());
        return;
    }

    moving_links_.push_back(link_name);
    poses_.push_back(pose);
    KdlTreeIk::NodePriority priority_vec;
    for (size_t i = 0; i < priority.size(); ++i) // Set each DOF to the given priority
        priority_vec[i] = priority[i];
    priorities_.push_back(priority_vec);
}

void TreeIkRequest::setWorldState(const Eigen::Affine3d& pose)
{
    Eigen::Vector3d rpy = pose.rotation().eulerAngles(0,1,2);
    Eigen::Vector3d trans = pose.translation();

    world_state_rpy_.resize(6);
    world_state_rpy_[0] = trans(0);
    world_state_rpy_[1] = trans(1);
    world_state_rpy_[2] = trans(2);
    world_state_rpy_[3] = rpy(0);
    world_state_rpy_[4] = rpy(1);
    world_state_rpy_[5] = rpy(2);
}

//// TreeIkResponse ////
bool TreeIkResponse::successful()                           const { return success_; }
const Eigen::Affine3d& TreeIkResponse::getWorldState()      const { return world_pose_; }
const std::vector<double>& TreeIkResponse::getJointValues() const { return joint_values_; }
void TreeIkResponse::setFailure()                                 { success_ = false; }
void TreeIkResponse::setValues(const Eigen::Affine3d& world, const std::vector<double>& joints)
{
    success_ = true;
    world_pose_ = world;
    joint_values_ = joints;
}


//// Kinematics Plugin ////
MoveItR2TreeKinematicsPlugin::MoveItR2TreeKinematicsPlugin () : ik_(NULL), fk_(NULL)
{
}

MoveItR2TreeKinematicsPlugin::~MoveItR2TreeKinematicsPlugin ()
{
    if (ik_)
        delete ik_;
    if (fk_)
        delete fk_;
}

// Total hack for RViz
bool MoveItR2TreeKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
    ROS_ERROR("GetPositionIK for one pose is NOT implemented in this solver");
    return false;
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    std::vector<double> &solution,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    const std::vector<double> &consistency_limits,
                                                    std::vector<double> &solution,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    std::vector<double> &solution,
                                                    const IKCallbackFn &solution_callback,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    const std::vector<double> &consistency_limits,
                                                    std::vector<double> &solution,
                                                    const IKCallbackFn &solution_callback,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

#define F_EQ(a, b) fabs(a - b) < 1e-4

static bool equalPoses(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    // position
    if (!F_EQ(pose1.position.x, pose2.position.x) ||
        !F_EQ(pose1.position.y, pose2.position.y) ||
        !F_EQ(pose1.position.z, pose2.position.z))
        return false;

    // orientation
    if (!F_EQ(pose1.orientation.x, pose2.orientation.x) ||
        !F_EQ(pose1.orientation.y, pose2.orientation.y) ||
        !F_EQ(pose1.orientation.z, pose2.orientation.z) ||
        !F_EQ(pose1.orientation.w, pose2.orientation.w))
        return false;

    return true;
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    const std::vector<double> &consistency_limits,
                                                    std::vector<double> &solution,
                                                    const IKCallbackFn &solution_callback,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options,
                                                    const moveit::core::RobotState* context_state) const
{
    if (ik_seed_state.size() != group_variable_count_)
    {
        ROS_ERROR("IK seed state size (%lu) does not equal the number of group variables (%u)", ik_seed_state.size(), group_variable_count_);
        return false;
    }

    if (!mobile_base_)
    {
        ROS_ERROR("This IK method is meant for a robot with a mobile base");
        return false;
    }

    // Creating the seed state
    robot_state::RobotStatePtr state(new robot_state::RobotState(robot_model_));
    double* joint_angles = state->getVariablePositions();
    for(size_t i = 0; i < ik_seed_state.size(); ++i)
        joint_angles[group_joint_to_robot_state_bijection_[i]] = ik_seed_state[i];
    state->update();  // forward kinematics

    TreeIkRequest request;
    TreeIkResponse response;

    // Hopefully one of the tips is at the same pose as the seed state
    std::string fixed_link("");
    std::string moving_link("");
    for(size_t i = 0; i < tip_frames_.size(); ++i)
    {
        // NOTE: The ik_poses are given in the coordinate frame of base_frame_.  Should probably transform the
        // pose below into this frame, in case the global frame is NOT the base_frame_.
        const Eigen::Affine3d& pose = state->getGlobalLinkTransform(tip_frames_[i]);
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        if (equalPoses(pose_msg, ik_poses[i]))
            request.addFixedLink(tip_frames_[i]);
        else
            request.addLinkPose(tip_frames_[i], ik_poses[i]);
    }

    // Not sure which link to set as the fixed base
    if (request.getFixedLinks().size() == 0)
    {
        ROS_ERROR("TreeKinematics: Fixed base link not found");
        return false;
    }
    if (request.getMovingLinks().size() == 0)
    {
        ROS_ERROR("TreeKinematics: No IK tasks");
        return false;
    }

    // Setting joint seed values for the IK request
    const std::vector<std::string>& all_joints = getAllJointNames();
    std::vector<double> joint_seed(total_dofs_);
    for(size_t i = 0; i < all_joints.size(); ++i)
        joint_seed[i] = joint_angles[ik_joints_to_robot_joints_bijection_[i]];
    request.setJointValues(joint_seed);

    // Posing the robot in space.  By convention, the floating joint is the last seven DOFs in the seed state.
    geometry_msgs::Pose world_pose_msg;
    world_pose_msg.position.x = ik_seed_state[group_variable_count_ - 7];
    world_pose_msg.position.y = ik_seed_state[group_variable_count_ - 6];
    world_pose_msg.position.z = ik_seed_state[group_variable_count_ - 5];
    world_pose_msg.orientation.x = ik_seed_state[group_variable_count_ - 4];
    world_pose_msg.orientation.y = ik_seed_state[group_variable_count_ - 3];
    world_pose_msg.orientation.z = ik_seed_state[group_variable_count_ - 2];
    world_pose_msg.orientation.w = ik_seed_state[group_variable_count_ - 1];

    Eigen::Affine3d world_pose;
    tf::poseMsgToEigen(world_pose_msg, world_pose);
    request.setWorldState(world_pose);

    // IK request
    if (getPositionIk(request, response))
    {
        solution.assign(response.getJointValues().begin(), response.getJointValues().end());

        // Add the world pose.  A bit of a hard-code here - the order of the joint
        // variables is assumed to be x,y,z, qx, qy, qz, qw.
        const Eigen::Affine3d& world_pose = response.getWorldState();

        solution.push_back(world_pose.translation()[0]);    // x
        solution.push_back(world_pose.translation()[1]);    // y
        solution.push_back(world_pose.translation()[2]);    // z

        Eigen::Quaternion<double> q(world_pose.rotation());
        solution.push_back(q.x());
        solution.push_back(q.y());
        solution.push_back(q.z());
        solution.push_back(q.w());

        return true;
    }
    return false;
}

bool MoveItR2TreeKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                                 const std::vector<double> &joint_angles,
                                                 std::vector<geometry_msgs::Pose> &poses) const
{
    poses.resize(link_names.size());
    if (joint_angles.size() != group_variable_count_)
    {
        ROS_ERROR("Joint angles vector must have size %u.  Received vector with size %lu", group_variable_count_, joint_angles.size());
        return false;
    }

    // Set the input joint values to the defaults
    // Change the joint values to those specified in link_names and joint_angles
    KDL::JntArray joints_in = default_joint_positions_;

    // Seeding input state
    // TODO: This is really slow.  Make faster with bijection.
    for(size_t i = 0; i < group_joints_.size(); ++i)
    {
        std::map<std::string, unsigned int>::const_iterator it;
        it = group_joint_index_map_.find(group_joints_[i]);

        if (it != group_joint_index_map_.end())
            joints_in(it->second) = joint_angles[i];
    }

    // Constructing map with names of frames we want
    std::map<std::string, KDL::Frame> frames;
    for(size_t i = 0; i < link_names.size(); ++i)
        frames[link_names[i]] = KDL::Frame();

    // Requesting frames
    bool valid = true;
    fk_mutex_.lock();
    try
    {
        fk_->getPoses(joints_in, frames);
    }
    catch (std::runtime_error e)
    {
        ROS_WARN("Exception caught during FK computation: %s", e.what());
        valid = false;
    }
    fk_mutex_.unlock();

    // Extracting pose information
    for (size_t i = 0; i < poses.size() && valid; ++i)
    {
        std::string link_name = (link_names[i][0] == '/' ? link_names[i] : "/" + link_names[i]);
        tf::poseKDLToMsg(frames[link_name], poses[i]);

        Eigen::Affine3d pose;
        tf::poseMsgToEigen(poses[i], pose);

        // Need to transform all frames into world frame (world frame is conveniently located in joint_angles)
        if (mobile_base_variable_count_ > 0)
        {
            unsigned int idx = joint_angles.size() - mobile_base_variable_count_;

            Eigen::Affine3d root_frame = Eigen::Affine3d::Identity();
            root_frame.translation()(0) = joint_angles[idx++];
            root_frame.translation()(1) = joint_angles[idx++];
            root_frame.translation()(2) = joint_angles[idx++];

            Eigen::Quaterniond q(joint_angles[idx+3], joint_angles[idx], joint_angles[idx+1], joint_angles[idx+2]);
            root_frame = root_frame * Eigen::Affine3d(q.toRotationMatrix());

            // One transformation please...
            pose = root_frame * pose;
            tf::poseEigenToMsg(pose, poses[i]);
        }
    }
    return valid;
}

bool MoveItR2TreeKinematicsPlugin::initialize(const std::string& robot_description,
                                              const std::string& group_name,
                                              const std::string& base_frame,
                                              const std::string& tip_frame,
                                              double search_discretization)
{
    std::vector<std::string> tip_frames;
    tip_frames.push_back(tip_frame);
    return initialize(robot_description, group_name, base_frame, tip_frames, search_discretization);
}

/// @brief Initialization function for the kinematics, for use with non-chain IK solvers
/// @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for;
/// For example, rhe name of the ROS parameter that contains the robot description;
/// @param group_name The group for which this solver is being configured
/// @param base_frame The base frame in which all input poses are expected.
/// This may (or may not) be the root frame of the chain that the solver operates on
/// @param tip_frames A vector of tips of the kinematic tree
/// @param search_discretization The discretization of the search when the solver steps through the redundancy
/// @return True if initialization was successful, false otherwise
bool MoveItR2TreeKinematicsPlugin::initialize(const std::string& robot_description,
                                              const std::string& group_name,
                                              const std::string& base_frame,
                                              const std::vector<std::string>& tip_frames,
                                              double search_discretization)
{
    // Consciously NOT using the provided setValues function in the base class
    // DO NOT USE THIS FUNCTION-----> setValues(robot_description, group_name, base_frame, tip_frame, search_discretization); <---- USE THIS AND BE SORRY
    //                                ^-- This method removes the leading slash from base_frame and tip_frame, which wreaks havoc on the R2 joint naming convention
    robot_description_ = robot_description;
    group_name_ = group_name;
    base_frame_ = base_frame;

    if (tip_frames.size() <= 1)
        ROS_WARN("Found %lu tip frame(s).  Did you add a kinematics_solver_ik_links entry to kinematics.yaml?", tip_frames.size());

    tip_frames_ = tip_frames;
    tip_frame_ = tip_frames[0]; // For legacy KinematicsBase functionality
    search_discretization_ = search_discretization;

    ROS_INFO("Initializing MoveItR2TreeKinematics for \"%s\"", group_name_.c_str());
    ROS_INFO("  Base frame: %s", base_frame_.c_str());
    ROS_INFO("  %lu Tip frames:", tip_frames_.size());
    for(size_t i = 0; i < tip_frames_.size(); ++i)
        ROS_INFO("  [%lu]: %s", i, tip_frames_[i].c_str());

    // Loading URDF and SRDF
    rdf_loader::RDFLoader rdf_loader(robot_description_);
    const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
    const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

    if (!urdf_model)
    {
        ROS_ERROR("URDF must be loaded for R2 kinematics solver to work.");
        return false;
    }
    if (!srdf)
    {
        ROS_ERROR("SRDF must be loaded for R2 kinematics solver to work.");
        return false;
    }

    // Load the robot kinematics and semantics (e.g., group information)
    robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

    // Extract the joint group
    if(!robot_model_->hasJointModelGroup(group_name_))
    {
        ROS_ERROR("Kinematic model does not contain group \"%s\"", group_name_.c_str());
        return false;
    }

    // Getting kinematic model for the joint group in question
    robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name_);
    group_variable_count_ = jmg->getVariableCount();

    ik_ = new MobileTreeIk();
    fk_ = new KdlTreeFk();

    // Attempt to initialize kinematics solvers with a simplified URDF, for efficiency
    ros::NodeHandle nh;
    bool initialized_kinematics = false;
    bool use_simplified_urdf = false;
    if (nh.hasParam(group_name_ + "/simplified_robot_description"))
    {
        try
        {
            // Initializing Ik and FK from URDF parameter
            fk_->loadFromParam(group_name_ + "/simplified_robot_description");
            ik_->loadFromParam(group_name_ + "/simplified_robot_description"); // 2.5x speedup on legs IK when excluding the upper body
            ROS_INFO("TreeKinematics: Loaded simplified URDF for group '%s'", group_name_.c_str());
            initialized_kinematics = true;
            use_simplified_urdf = true;
        }
        catch (std::runtime_error) {} // if the simplified_robot_description doesn't exist, a runtime error will be thrown
    }

    // No simplified URDF found, or there was an error loading simplified URDF
    if (!initialized_kinematics)
    {
        try
        {
            fk_->loadFromParam(robot_description_);
            ik_->loadFromParam(robot_description_);
        }
        catch (std::runtime_error)
        {
            return false;
        }
    }

    ik_->getJointNames(joint_names_);  // reading in ALL joint names from IK representation

    const std::vector<const robot_model::JointModel*>& joint_roots = jmg->getJointRoots();
    mobile_base_ = false;
    mobile_base_variable_count_ = 0;
    std::string base_joint_ = "";
    for (size_t i = 0; i < joint_roots.size(); ++i)
    {
        if (joint_roots[i]->getType() == robot_model::JointModel::FLOATING)
        {
            base_joint_ = joint_roots[i]->getName();
            mobile_base_ = true;
            mobile_base_variable_count_ = joint_roots[i]->getVariableCount();
         }
    }

    if (!mobile_base_)
        ROS_WARN("TreeKinematics: Expected a floating root joint");

    // Getting default joint values
    std::map<std::string, double> default_joints;
    std::map<std::string, double> joint_inertias;
    robot_model_->getVariableDefaultPositions(default_joints);

    // The total number of DOF, minus floating root
    if (use_simplified_urdf)
        total_dofs_ = joint_names_.size();
    else
        total_dofs_ = default_joints.size() - mobile_base_variable_count_;
    default_joint_positions_.resize(total_dofs_);

    //ROS_INFO("Initializing MobileTreeIK for \"%s\" with %u DOFs", group_name_.c_str(), total_dofs_);

    std::vector<double> min_joint_limits;
    std::vector<double> max_joint_limits;
    // Extracting all joint limits from URDF and joints in the group
    // TODO: Read joint limits from MoveIt yaml config instead?
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_[i]);
        std::pair<double, double> limits(bounds.min_position_, bounds.max_position_);

        //ROS_INFO("    [%lu] - %s with range [%1.4f, %1.4f]", i, joint_names_[i].c_str(), limits.first, limits.second);
        //ROS_INFO("            Attached to link: %s", robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName().c_str());

        // Inserting into joint limit maps
        min_joint_limits.push_back(limits.first);
        max_joint_limits.push_back(limits.second);

        // Unit mass for all joints.  TODO: Fill this with the actual value.
        joint_inertias[joint_names_[i]] = 1.0;

        // Save default joint value
        default_joint_positions_(i) = default_joints[joint_names_[i]];

        // Constructing bijection between joint value array for internal ik and joint value array for MoveIt robot state.
        const robot_model::JointModel* joint = robot_model_->getJointModel(joint_names_[i]);
        const std::vector<std::string>& var_names = joint->getVariableNames();
        for(size_t j = 0; j < var_names.size(); ++j)
            ik_joints_to_robot_joints_bijection_.push_back(robot_model_->getVariableIndex(var_names[j]));

        // This joint is in the group
        if (jmg->hasJointModel(joint_names_[i]))
        {
            group_joint_index_map_[joint_names_[i]] = i;
            group_joints_.push_back(joint_names_[i]);
            group_links_.push_back(robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName());

            // Constructing bijection between joint value array for the group and joint value array for MoveIt robot state.
            for(size_t j = 0; j < var_names.size(); ++j)
                group_joint_to_robot_state_bijection_.push_back(robot_model_->getVariableIndex(var_names[j]));
        }
    }

    // Adding the floating (virtual) joint to the group joints list since it is passively actuated
    // By design, this joint and link is always the last entry in the group_joints and group_links lists.
    if (mobile_base_)
    {
        const robot_model::JointModel* joint = robot_model_->getJointModel(base_joint_);

        group_joints_.push_back(base_joint_);
        group_links_.push_back(joint->getChildLinkModel()->getName());

        const std::vector<std::string>& var_names = joint->getVariableNames();
        for(size_t j = 0; j < var_names.size(); ++j)
            group_joint_to_robot_state_bijection_.push_back(robot_model_->getVariableIndex(var_names[j]));
    }

    if(group_joints_.size() != group_links_.size())
    {
        ROS_ERROR("TreeKinematics: The number of joints is not equal to the number of links!");
        return false;
    }
    if(group_joint_to_robot_state_bijection_.size() != group_variable_count_)
    {
        ROS_ERROR("TreeKinematics: Bijection size mismatch");
        return false;
    }

    // Initializing position limiter (joint limits) for MobileTreeIk
    boost::shared_ptr<JointNamePositionLimiter> position_limiter(new JointNamePositionLimiter());
    position_limiter->setJointPositionLimiter(boost::shared_ptr<JointPositionLimiter>(new JointPositionLimiter()));
    position_limiter->setLimits(joint_names_, min_joint_limits, max_joint_limits);

    // Set IK joint limits and inertias
    ik_->setPositionLimiter(position_limiter);
    ik_->setJointInertias(joint_inertias);

    // Construct tolerance map for different priorities
    std::map<int, std::pair<double, double> > priority_tol; // linear and angular tolerances for each priority level
    priority_tol[KdlTreeIk::CRITICAL] = std::make_pair<double, double>(CRITICAL_PRIO_LINEAR_TOL, CRITICAL_PRIO_ANGULAR_TOL);
    priority_tol[KdlTreeIk::HIGH] = std::make_pair<double, double>(HIGH_PRIO_LINEAR_TOL, HIGH_PRIO_ANGULAR_TOL);
    priority_tol[KdlTreeIk::MEDIUM] = std::make_pair<double, double>(MEDIUM_PRIO_LINEAR_TOL, MEDIUM_PRIO_ANGULAR_TOL);
    priority_tol[KdlTreeIk::LOW] = std::make_pair<double, double>(LOW_PRIO_LINEAR_TOL, LOW_PRIO_ANGULAR_TOL);
    ik_->setPriorityTol(priority_tol);
    ik_->setMaxJointVel(MAX_JOINT_VELOCITY);
    ik_->setMaxTwist(MAX_TWIST);
    ik_->setMBar(1e-12);

    return true;
}

const std::vector<std::string>& MoveItR2TreeKinematicsPlugin::getJointNames() const
{
    return group_joints_;
}

const std::vector<std::string>& MoveItR2TreeKinematicsPlugin::getLinkNames() const
{
    return group_links_;
}

const bool MoveItR2TreeKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
                                                       std::string* error_text_out) const
{
    // We can do kinematics for virtually anything.
    return true;
}

const std::vector<std::string>& MoveItR2TreeKinematicsPlugin::getAllJointNames() const
{
    return joint_names_;
}

bool MoveItR2TreeKinematicsPlugin::getPositionIk(const TreeIkRequest& request, TreeIkResponse& response) const
{
    // convert geometry_msgs::Pose into KDL frames
    std::vector<KDL::Frame> frames;
    const std::vector<geometry_msgs::Pose>& poses = request.getMovingLinkPoses();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        KDL::Frame frame;
        tf::poseMsgToKDL(poses[i], frame);
        frames.push_back(frame);
    }

    const std::vector<double>& seed_joints = request.getJointValues();
    if (seed_joints.size() != default_joint_positions_.rows())
    {
        ROS_ERROR("# joints in seed (%lu) not equal to the number of DOF (%u) in system", seed_joints.size(), default_joint_positions_.rows());
        response.setFailure();
        return false;
    }

    if (request.getFixedLinks().size() == 0)
    {
        ROS_ERROR("No fixed base links specified");
        response.setFailure();
        return false;
    }

    KDL::JntArray joints_in = default_joint_positions_;
    // Update given ik_seed
    for (size_t i = 0; i < seed_joints.size(); ++i)
        joints_in(i) = seed_joints[i];

    KDL::JntArray joints_out;
    joints_out.resize(total_dofs_);

    std::vector<double> world_joints; // store the resulting world DOFs here

    // NOTE: For some horrible reason, boost::mutex::scoped_lock does NOT actually acquire ik_mutex_.
    // MobileTreeIK is NOT threadsafe!
    ik_mutex_.lock();

    try
    {
        // Set fixed bases
        ik_->setBases(request.getFixedLinks());

        // Update world state
        ik_->setMobileJoints(request.getWorldStateRPY());

        // Performing IK
        ik_->getJointPositions(joints_in, request.getMovingLinks(), frames, joints_out, request.getPriorities());
    }
    catch (std::runtime_error e) // MobileTreeIk throws an exception when IK fails.
    {
        //ROS_ERROR("IK Failed - %s", e.what());
        response.setFailure();
        ik_mutex_.unlock();
        return false;
    }

    // fill out response
    ik_->getMobileJoints(world_joints);

    // We no longer need the lock
    ik_mutex_.unlock();

    // Getting the values of the passive DOFs (the world pose)
    Eigen::Affine3d world_pose;
    world_pose.translation()[0] = world_joints[0];
    world_pose.translation()[1] = world_joints[1];
    world_pose.translation()[2] = world_joints[2];

    world_pose = Eigen::Translation3d(world_pose.translation()) *
                (Eigen::AngleAxisd(world_joints[3], Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(world_joints[4], Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(world_joints[5], Eigen::Vector3d::UnitZ()));

    // Construct solution vector of joint positions
    std::vector<double> solution;
    for (unsigned int i = 0; i < group_joints_.size() - (mobile_base_ ? 1 : 0); ++i) // -1 excludes the floating multi-dof root joint.  This joint is ALWAYS added last
    {
        unsigned int idx = group_joint_index_map_.find(group_joints_[i])->second;
        solution.push_back(joints_out(idx));
    }

    response.setValues(world_pose, solution);
    return true;
}

} // namespace moveit_r2_kinematics
