/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#ifndef MOVEIT_R2_TREE_KINEMATICS_TOLERANCES_
#define MOVEIT_R2_TREE_KINEMATICS_TOLERANCES_

namespace moveit_r2_kinematics
{
    /// \brief Critical (highest) priority task linear tolerance
    const double CRITICAL_PRIO_LINEAR_TOL = 1e-5;
    /// \brief Critical (highest) priority task angular tolerance
    const double CRITICAL_PRIO_ANGULAR_TOL = 1e-5;

    /// \brief High priority task linear tolerance
    const double HIGH_PRIO_LINEAR_TOL = 1e-1;
    /// \brief High priority task angular tolerance
    const double HIGH_PRIO_ANGULAR_TOL = 1e-1;

    /// \brief Medium priority task linear tolerance
    const double MEDIUM_PRIO_LINEAR_TOL = 1.0;
    /// \brief Medium priority task angular tolerance
    const double MEDIUM_PRIO_ANGULAR_TOL = 1.0;

    /// \brief Low priority task linear tolerance
    const double LOW_PRIO_LINEAR_TOL = 3.0;
    /// \brief Low priority task angular tolerance
    const double LOW_PRIO_ANGULAR_TOL = 3.0;

    /// \brief The maximum allowed joint rotational velocity
    const double MAX_JOINT_VELOCITY = 5.0;

    /// \brief The maximum allowed twist
    const double MAX_TWIST = 0.3;
}

#endif