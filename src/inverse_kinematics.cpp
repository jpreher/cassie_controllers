/*
 * MIT License
 * 
 * Copyright (c) 2020 Jenna Reher (jreher@caltech.edu)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <cassie_controllers/inverse_kinematics.hpp>

using namespace Eigen;
using namespace cassie_model;

namespace cassie_inverse_kinematics {

StandingInverseKinematics::StandingInverseKinematics() {
    this->target.resize(10);
}

void StandingInverseKinematics::init(cassie_model::Cassie &robot) {
    this->robot = &robot;
}

int StandingInverseKinematics::operator()(const VectorXd &x, VectorXd &fvec) const
{
    // 'x' has dimensions n x 1
    // It contains the current estimates for the parameters.
    VectorXd q(22);
    q << this->robot->q;
    for (int i=0; i<6; ++i)
        q(i) = 0.0;
    for (int i=0; i<this->robot->iRotorMap.size(); ++i)
        q(this->robot->iRotorMap(i)) = x(i);
    q(LeftShinPitch)    = 0.0;
    q(LeftTarsusPitch)  = 0.226893 - q(LeftKneePitch); // 13 deg - knee
    q(RightShinPitch)   = 0.0;
    q(RightTarsusPitch) = 0.226893 - q(RightKneePitch);

    MatrixXd pose_leftFoot(6,1), pose_rightFoot(6,1);
    MatrixXd position_leftFoot(3,1), position_rightFoot(3,1);

    SymFunction::pose_leftFoot(pose_leftFoot, q);
    SymFunction::pose_rightFoot(pose_rightFoot, q);
    SymFunction::p_leftToe(position_leftFoot, q);
    SymFunction::p_rightToe(position_rightFoot, q);

    VectorXd F(10);
    F << position_leftFoot(0,0),  position_leftFoot(1,0),  position_leftFoot(2,0),  pose_leftFoot(4,0),  pose_leftFoot(5,0),
         position_rightFoot(0,0), position_rightFoot(1,0), position_rightFoot(2,0), pose_rightFoot(4,0), pose_rightFoot(5,0);

    fvec = F - this->target;

    return 0;
}

int StandingInverseKinematics::df(const VectorXd &x, MatrixXd &fjac) const
{
    // 'x' has dimensions n x 1
    // It contains the current estimates for the parameters.
    VectorXd q(22);
    q << robot->q;
    for (int i=0; i<6; ++i)
        q(i) = 0.0;
    for (int i=0; i<this->robot->iRotorMap.size(); ++i)
        q(this->robot->iRotorMap(i)) = x(i);
    q(CassieStateEnum::LeftShinPitch)  = 0.0;
    q(CassieStateEnum::RightShinPitch) = 0.0;
    q(CassieStateEnum::LeftTarsusPitch)  = 0.226893 - q(CassieStateEnum::LeftKneePitch);
    q(CassieStateEnum::RightTarsusPitch) = 0.226893 - q(CassieStateEnum::RightKneePitch);

    // 'fjac' has dimensions m x n
    // It will contain the jacobian of the errors.
    MatrixXd J_poseLeft(6,22), J_poseRight(6,22);
    MatrixXd J_positionLeft(3,22), J_positionRight(3,22);

    SymFunction::J_leftFoot(J_poseLeft, q);
    SymFunction::J_rightFoot(J_poseRight, q);
    SymFunction::J_leftToe(J_positionLeft, q);
    SymFunction::J_rightToe(J_positionRight, q);

    for (int i=0; i<3; ++i) {
        J_poseLeft.row(i)  = J_positionLeft.row(i);
        J_poseRight.row(i) = J_positionRight.row(i);
    }

    fjac.row(0) << J_poseLeft(0,LeftHipRoll), 0.0, J_poseLeft(0,LeftHipPitch), J_poseLeft(0,LeftKneePitch) - J_poseLeft(0,LeftTarsusPitch), 0.0,     0.0, 0.0, 0.0, 0.0, 0.0;
    fjac.row(1) << J_poseLeft(1,LeftHipRoll), 0.0, J_poseLeft(1,LeftHipPitch), J_poseLeft(1,LeftKneePitch) - J_poseLeft(1,LeftTarsusPitch), 0.0,     0.0, 0.0, 0.0, 0.0, 0.0;
    fjac.row(2) << J_poseLeft(2,LeftHipRoll), 0.0, J_poseLeft(2,LeftHipPitch), J_poseLeft(2,LeftKneePitch) - J_poseLeft(2,LeftTarsusPitch), 0.0,     0.0, 0.0, 0.0, 0.0, 0.0;
    fjac.row(3) << 0.0, 0.0, 0.0, 0.0, -1.0,         0.0, 0.0, 0.0, 0.0, 0.0;
    fjac.row(4) << 0.0, 1.0, 0.0, 0.0,  0.0,         0.0, 0.0, 0.0, 0.0, 0.0;
    //fjac.row(3) << J_poseLeft(4,0), J_poseLeft(4,1), J_poseLeft(4,2), J_poseLeft(4,3) - J_poseLeft(4,5), J_poseLeft(4,6), 0.0, 0.0, 0.0, 0.0, 0.0;
    //fjac.row(4) << J_poseLeft(5,0), J_poseLeft(5,1), J_poseLeft(5,2), J_poseLeft(5,3) - J_poseLeft(5,5), J_poseLeft(5,6), 0.0, 0.0, 0.0, 0.0, 0.0;

    fjac.row(5) << 0.0, 0.0, 0.0, 0.0, 0.0,     J_poseRight(0,RightHipRoll), 0.0, J_poseRight(0,RightHipPitch), J_poseRight(0,RightKneePitch) - J_poseRight(0,RightTarsusPitch), 0.0;
    fjac.row(6) << 0.0, 0.0, 0.0, 0.0, 0.0,     J_poseRight(1,RightHipRoll), 0.0, J_poseRight(1,RightHipPitch), J_poseRight(1,RightKneePitch) - J_poseRight(1,RightTarsusPitch), 0.0;
    fjac.row(7) << 0.0, 0.0, 0.0, 0.0, 0.0,     J_poseRight(2,RightHipRoll), 0.0, J_poseRight(2,RightHipPitch), J_poseRight(2,RightKneePitch) - J_poseRight(2,RightTarsusPitch), 0.0;
    fjac.row(8) << 0.0, 0.0, 0.0, 0.0, 0.0,     0.0, 0.0, 0.0, 0.0, -1.0;
    fjac.row(9) << 0.0, 0.0, 0.0, 0.0, 0.0,     0.0, 1.0, 0.0, 0.0, 0.0;
    //fjac.row(8) << 0.0, 0.0, 0.0, 0.0, 0.0,   J_poseRight(4,0), J_poseRight(4,1), J_poseRight(4,2), J_poseRight(4,3) - J_poseRight(4,5), J_poseRight(4,6);
    //fjac.row(9) << 0.0, 0.0, 0.0, 0.0, 0.0,   J_poseRight(5,0), J_poseRight(5,1), J_poseRight(5,2), J_poseRight(5,3) - J_poseRight(5,5), J_poseRight(5,6);

    return 0;
}

void StandingInverseKinematics::setTarget(Transform<double, 3, Affine> &T_pelvis, Transform<double, 3, Affine> &T_lf, double pitchLF, Transform<double, 3, Affine> &T_rf, double pitchRF) {
    this->target << T_pelvis.rotation().transpose() * T_lf.translation(),
                    pitchLF,
                    0.0,
                    T_pelvis.rotation().transpose() * T_rf.translation(),
                    pitchRF,
                    0.0;
}



}
