/* Author: Ryan Luna */

#ifndef MOVEIT_R2_TREE_KINEMATICS_PLUGIN_
#define MOVEIT_R2_TREE_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// NASA R2
#include <nasa_robodyn_controllers_core/MobileTreeIk.h>
#include <nasa_robodyn_controllers_core/KdlTreeFk.h>

#include <boost/thread/mutex.hpp>

namespace moveit_r2_kinematics
{
    /// \brief Custom structure for IK requests on a kinematic tree.
    class TreeIkRequest
    {
    public:
        /// \brief Set the given link as fixed in the IK request.  This link will serve as a base for the IK solver.
        void addFixedLink(const std::string& link_name);
        /// \brief Add an IK task to move link_name to the given pose (in global coordinates) with the given priority
        void addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const int priority=KdlTreeIk::CRITICAL);
        /// \brief Add an IK task to move link_name to the given pose (in global coordinates) with the given priority
        void addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const std::vector<int>& priority);
        /// \brief Set the pose of the robot in the world.  This pose corresponds to the "mobile joints" of the IK solver.
        void setWorldState(const Eigen::Affine3d& pose);
        /// \brief Set the current (seed) values for the robot.
        void setJointValues(const std::vector<double>& values);

        const std::vector<std::string>& getFixedLinks() const;
        const std::vector<std::string>& getMovingLinks() const;
        const std::vector<geometry_msgs::Pose>& getMovingLinkPoses() const;
        const std::vector<double>& getJointValues() const;
        const geometry_msgs::Pose& getWorldState() const;
        const std::vector<double>& getWorldStateRPY() const;
        const std::vector<KdlTreeIk::NodePriority>& getPriorities() const;

    protected:
        std::vector<std::string> fixed_links_;
        std::vector<std::string> moving_links_;
        std::vector<geometry_msgs::Pose> poses_;
        std::vector<KdlTreeIk::NodePriority> priorities_;

        std::vector<double> initial_joints_;
        std::vector<double> world_state_rpy_;
        geometry_msgs::Pose world_state_;
    };

    /// \brief Custom structure for an IK response
    class TreeIkResponse
    {
    public:
        /// \brief True if the IK request was successful.
        bool successful() const;
        /// \brief Return the new pose of the robot in the world.  This pose corresponds to the "mobile joints" of the IK solver.
        const Eigen::Affine3d& getWorldState() const;
        /// \brief Return the new joint values for the robot
        const std::vector<double>& getJointValues() const;

        /// \brief Mark the IK request as a failure
        void setFailure();
        /// \brief Set the world state and joint values after a successful IK request
        void setValues(const Eigen::Affine3d& world, const std::vector<double>& joints);

    protected:
        bool success_;
        Eigen::Affine3d world_pose_;
        std::vector<double> joint_values_;
    };

    /// \brief Custom kinematics routines for R2.  NOTE: Currently this implementation is only be functional on the legs.  See implementation of initialize(...).
    class MoveItR2TreeKinematicsPlugin : public kinematics::KinematicsBase
    {
    public:
        MoveItR2TreeKinematicsPlugin();
        virtual ~MoveItR2TreeKinematicsPlugin ();

        /// @group KinematicsBase interface
        /// @{

        /// @brief Given a desired pose of the end-effector, compute the joint angles to reach it
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param solution the solution vector
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   std::vector<double> &solution,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param solution the solution vector
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
        /// @param solution the solution vector
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param solution the solution vector
        /// @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
        /// @param solution_callback A callback solution for the IK solution
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
        /// @param solution the solution vector
        /// @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
        /// @param solution_callback A callback solution for the IK solution
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options) const;

        /// @brief Given a set of desired poses for a planning group with multiple end-effectors, search for the joint angles
        ///        required to reach them. This is useful for e.g. biped robots that need to perform whole-body IK.
        ///        Not necessary for most robots that have kinematic chains.
        ///        This particular method is intended for "searching" for a solutions by stepping through the redundancy
        ///        (or other numerical routines).
        /// @param ik_poses the desired pose of each tip link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
        /// @param solution the solution vector
        /// @param solution_callback A callback solution for the IK solution
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param options container for other IK options
        /// @param context_state (optional) the context in which this request
        ///        is being made.  The position values corresponding to
        ///        joints in the current group may not match those in
        ///        ik_seed_state.  The values in ik_seed_state are the ones
        ///        to use.  This is passed just to provide the \em other
        ///        joint values, in case they are needed for context, like
        ///        with an IK solver that computes a balanced result for a
        ///        biped.
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
                                const moveit::core::RobotState* context_state = NULL) const;

        /// @brief Given a set of joint angles and a set of links, compute their pose
        /// @param link_names A set of links for which FK needs to be computed
        /// @param joint_angles The state for which FK is being computed
        /// @param poses The resultant set of poses (in the frame returned by getBaseFrame())
        /// @return True if a valid solution was found, false otherwise
        virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                   const std::vector<double> &joint_angles,
                                   std::vector<geometry_msgs::Pose> &poses) const;

        /// @brief  Initialization function for the kinematics
        /// @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for; For example, rhe name of the ROS parameter that contains the robot description;
        /// @param group_name The group for which this solver is being configured
        /// @param base_frame The base frame in which all input poses are expected.
        /// This may (or may not) be the root frame of the chain that the solver operates on
        /// @param tip_frame The tip of the chain
        /// @param search_discretization The discretization of the search when the solver steps through the redundancy
        /// @return True if initialization was successful, false otherwise
        virtual bool initialize(const std::string& robot_description,
                                const std::string& group_name,
                                const std::string& base_frame,
                                const std::string& tip_frame,
                                double search_discretization);

        /// @brief Initialization function for the kinematics, for use with non-chain IK solvers
        /// @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for;
        /// For example, rhe name of the ROS parameter that contains the robot description;
        /// @param group_name The group for which this solver is being configured
        /// @param base_frame The base frame in which all input poses are expected.
        /// This may (or may not) be the root frame of the chain that the solver operates on
        /// @param tip_frames A vector of tips of the kinematic tree
        /// @param search_discretization The discretization of the search when the solver steps through the redundancy
        /// @return True if initialization was successful, false otherwise
        virtual bool initialize(const std::string& robot_description,
                                const std::string& group_name,
                                const std::string& base_frame,
                                const std::vector<std::string>& tip_frames,
                                double search_discretization);

        /// @brief Return all the joint names in the order they are used internally
        virtual const std::vector<std::string>& getJointNames() const;

        /// @brief Return all the link names in the order they are represented internally
        virtual const std::vector<std::string>& getLinkNames() const;

        /**
        * \brief Check if this solver supports a given JointModelGroup.
        *
        * Override this function to check if your kinematics solver
        * implementation supports the given group.
        *
        * The default implementation just returns jmg->isChain(), since
        * solvers written before this function was added all supported only
        * chain groups.
        *
        * \param jmg the planning group being proposed to be solved by this IK solver
        * \param error_text_out If this pointer is non-null and the group is
        *          not supported, this is filled with a description of why it's not
        *          supported.
        * \return True if the group is supported, false if not.
        */
        virtual const bool supportsGroup(const moveit::core::JointModelGroup *jmg,
                                         std::string* error_text_out = NULL) const;

        /// @brief Redundant joints are disallowed in this kinematics solver
        virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices) { return false; }

        /// @}

        // Custom interface
        // Tree IK request and response
        virtual bool getPositionIk(const TreeIkRequest& request, TreeIkResponse& response) const;
        // Return the order of ALL actuable joints in the entire system, not just the group
        virtual const std::vector<std::string>& getAllJointNames() const;

    protected:
        /// \brief A model of the entire robot
        robot_model::RobotModelPtr robot_model_;

        /// \brief True if the robot is mobile (the robot is not fixed in place)
        bool mobile_base_;

        /// \brief The IK solver
        MobileTreeIk* ik_;
        /// \brief The FK solver
        KdlTreeFk* fk_;

        /// \brief Mutex locking access to the ik_ member
        mutable boost::mutex ik_mutex_;
        /// \brief Mutex locking access to the fk_ member
        mutable boost::mutex fk_mutex_;

        /// \brief The default joint position for ALL joints in R2
        KDL::JntArray default_joint_positions_;

        /// \brief The number of actuable DOFs in R2
        unsigned int total_dofs_;
        /// \brief The number of passive DOFs in the mobile base joint
        unsigned int mobile_base_variable_count_;
        /// \brief The number of variables in the IK joint group (actuable and passive)
        unsigned int group_variable_count_;

        /// \brief A mapping of joints in the group to their index in jointNames_, default_joint_positions_, etc..
        std::map<std::string, unsigned int> group_joint_index_map_;

        /// \brief The set of joints to perform IK for.
        std::vector<std::string> group_joints_;
        /// \brief The set of links to perform IK for.
        std::vector<std::string> group_links_;
        /// \brief A list of every joint in the system
        std::vector<std::string> joint_names_;

        /// \brief A mapping of group joint variables to their indices in the full robot state
        std::vector<int> group_joint_to_robot_state_bijection_;
        /// \brief A mapping of joint variables in the IK representation to their indices in the full robot state
        std::vector<int> ik_joints_to_robot_joints_bijection_;
  };
}

#endif
