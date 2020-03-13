/**
    @author Jenna Reher (jreher@caltech.edu)
*/

#ifndef CASSIE_FF_CONGTROLLER_HPP
#define CASSIE_FF_CONGTROLLER_HPP

#include <cassie_description/cassie_model.hpp>
#include <unsupported/Eigen/EulerAngles>

class feedforward_including_feet {
public:
    cassie_model::Cassie *robot;
    Eigen::VectorXd u;
    double weight = 326.8692;

    struct Cache {
        Eigen::VectorXd q_ff;
        Eigen::VectorXd GRF_desired;
        Eigen::VectorXd tau;
        Eigen::VectorXd p_lf;
        Eigen::VectorXd p_rf;

        Eigen::MatrixXd Jl;
        Eigen::MatrixXd Jr;
        Eigen::MatrixXd Jach;
        Eigen::MatrixXd Jlf_full;
        Eigen::MatrixXd Jrf_full;
        Eigen::MatrixXd Jlf;
        Eigen::MatrixXd Jrf;
        Eigen::MatrixXd Jach_full;
        Eigen::MatrixXd Jach_lf;
        Eigen::MatrixXd Jach_rf;
        Eigen::MatrixXd J_ff;

        void init() {
            q_ff.resize(22);
            GRF_desired.resize(6);
            tau.resize(14);
            p_lf.resize(3);
            p_rf.resize(3);

            Jl.resize(3,7);
            Jr.resize(3,7);
            Jach.resize(1,7);
            Jlf_full.resize(3,7);
            Jrf_full.resize(3,7);
            Jlf.resize(3,8);
            Jrf.resize(3,8);
            Jach_full.resize(2,22);
            Jach_lf.resize(1,8);
            Jach_rf.resize(1,8);
            J_ff.resize(6,14);

            this->reset();
        }

        void reset() {
            q_ff.setZero();
            GRF_desired.setZero();
            tau.setZero();
            p_lf.setZero();
            p_rf.setZero();

            Jl.setZero();
            Jr.setZero();
            Jach.setZero();
            Jlf_full.setZero();
            Jrf_full.setZero();
            Jlf.setZero();
            Jrf.setZero();
            Jach_full.setZero();
            Jach_lf.setZero();
            Jach_rf.setZero();
            J_ff.setZero();
        }

    } cache;

    feedforward_including_feet(cassie_model::Cassie &robot) {
        this->robot = &robot;
        this->u = Eigen::VectorXd::Zero(10);
        this->cache.init();
    }

    Eigen::VectorXd compute(Eigen::Transform<double, 3, Affine> &T_pelvis, Eigen::VectorXd &q_motor_desired, double s=0.0) {
        // make a pointer to the cache for name being too long
        feedforward_including_feet::Cache *ca = &this->cache;

        ca->q_ff.setZero();

        for (int i=0; i<6; ++i) {
            ca->q_ff(i) = 0.0;
        }

        // Step through the 10 motors and command them
        int ind = 0;
        for (int i=0; i<robot->iRotorMap.size(); ++i) {
            ind = robot->iRotorMap(i);
            ca->q_ff(ind) = q_motor_desired(i);
        }

        // Zero the springs
        double rad_13_deg = 0.226893;
        ca->q_ff(robot->iJointMap(0)) = 0.0;
        ca->q_ff(robot->iJointMap(1)) = rad_13_deg - q_motor_desired(3);
        ca->q_ff(robot->iJointMap(2)) = 0.0;
        ca->q_ff(robot->iJointMap(3)) = rad_13_deg - q_motor_desired(8);
        ca->q_ff(LeftHeelSpring) = 0.0;
        ca->q_ff(RightHeelSpring) = 0.0;

        this->robot->kinematics.computeConstrainedFootJacobian(ca->q_ff, ca->Jl, ca->Jr);

        ca->J_ff.block(0,0,3,7) << ca->Jl;
        ca->J_ff.block(3,7,3,7) << ca->Jr;

        // Desired GRF profiles
        Eigen::VectorXd dqLF(7), dqRF(7);
        dqLF << this->robot->dq(LeftHipRoll),
                this->robot->dq(LeftHipYaw),
                this->robot->dq(LeftHipPitch),
                this->robot->dq(LeftKneePitch),
                this->robot->dq(LeftShinPitch),
                this->robot->dq(LeftTarsusPitch),
                this->robot->dq(LeftFootPitch);
        dqRF << this->robot->dq(RightHipRoll),
                this->robot->dq(RightHipYaw),
                this->robot->dq(RightHipPitch),
                this->robot->dq(RightKneePitch),
                this->robot->dq(RightShinPitch),
                this->robot->dq(RightTarsusPitch),
                this->robot->dq(RightFootPitch);

        ca->GRF_desired << 0.0, 0.0, -this->weight * (1.0 - s) / 2.0,
                           0.0, 0.0, -this->weight * (1.0 + s) / 2.0;



        Matrix3d Rot = Eigen::EulerAnglesZYXd(0.0, this->robot->q(4), this->robot->q(3)).toRotationMatrix();
        ca->GRF_desired.block(0,0,3,1) << Rot * T_pelvis.rotation().transpose() * ca->GRF_desired.block(0,0,3,1);
        ca->GRF_desired.block(3,0,3,1) << Rot * T_pelvis.rotation().transpose() * ca->GRF_desired.block(3,0,3,1);

        // Compute
        ca->tau = ca->J_ff.transpose() * ca->GRF_desired;
        this->u << ca->tau(0), 0, ca->tau(2), ca->tau(3), ca->tau(6),
                   ca->tau(7), 0, ca->tau(9), ca->tau(10), ca->tau(13);

        return this->u;
    }
};

class feedforward_underactuated {
public:
    cassie_model::Cassie *robot;
    Eigen::VectorXd u;
    double weight = 326.8692;

    struct Cache {
        Eigen::VectorXd q_ff;
        Eigen::VectorXd GRF_desired;
        Eigen::VectorXd tau;
        Eigen::VectorXd p_lf;
        Eigen::VectorXd p_rf;

        Eigen::MatrixXd Jl;
        Eigen::MatrixXd Jr;
        Eigen::MatrixXd Jach;
        Eigen::MatrixXd Jlf_full;
        Eigen::MatrixXd Jrf_full;
        Eigen::MatrixXd Jlf;
        Eigen::MatrixXd Jrf;
        Eigen::MatrixXd Jach_full;
        Eigen::MatrixXd Jach_lf;
        Eigen::MatrixXd Jach_rf;
        Eigen::MatrixXd J_ff;

        void init() {
            q_ff.resize(22);
            GRF_desired.resize(6);
            tau.resize(14);
            p_lf.resize(3);
            p_rf.resize(3);

            Jl.resize(3,7);
            Jr.resize(3,7);
            Jach.resize(1,7);
            Jlf_full.resize(3,7);
            Jrf_full.resize(3,7);
            Jlf.resize(3,8);
            Jrf.resize(3,8);
            Jach_full.resize(2,22);
            Jach_lf.resize(1,8);
            Jach_rf.resize(1,8);
            J_ff.resize(6,14);

            this->reset();
        }

        void reset() {
            q_ff.setZero();
            GRF_desired.setZero();
            tau.setZero();
            p_lf.setZero();
            p_rf.setZero();

            Jl.setZero();
            Jr.setZero();
            Jach.setZero();
            Jlf_full.setZero();
            Jrf_full.setZero();
            Jlf.setZero();
            Jrf.setZero();
            Jach_full.setZero();
            Jach_lf.setZero();
            Jach_rf.setZero();
            J_ff.setZero();
        }

    } cache;

    feedforward_underactuated(cassie_model::Cassie &robot) {
        this->robot = &robot;
        this->u = Eigen::VectorXd::Zero(10);
        this->cache.init();
    }

    Eigen::VectorXd compute(Eigen::Transform<double, 3, Affine> &T_pelvis, Eigen::VectorXd &q_motor_desired, double s=0.0) {
        // make a pointer to the cache for name being too long
        feedforward_underactuated::Cache *ca = &this->cache;

        ca->q_ff.setZero();

        for (int i=0; i<6; ++i) {
            ca->q_ff(i) = 0.0;
        }

        // Step through the 10 motors and command them
        int ind = 0;
        for (int i=0; i<robot->iRotorMap.size(); ++i) {
            ind = robot->iRotorMap(i);
            ca->q_ff(ind) = q_motor_desired(i);
        }

        // Zero the springs
        double rad_13_deg = 0.226893;
        ca->q_ff(robot->iJointMap(0)) = 0.0;
        ca->q_ff(robot->iJointMap(1)) = rad_13_deg - q_motor_desired(3);
        ca->q_ff(robot->iJointMap(2)) = 0.0;
        ca->q_ff(robot->iJointMap(3)) = rad_13_deg - q_motor_desired(8);
        ca->q_ff(LeftHeelSpring) = 0.0;
        ca->q_ff(RightHeelSpring) = 0.0;

        this->robot->kinematics.computeConstrainedToeJacobian(ca->q_ff, ca->Jl, ca->Jr);
        ca->J_ff.block(0,0,3,7) << ca->Jl;
        ca->J_ff.block(3,7,3,7) << ca->Jr;

        // Desired GRF profiles
        Eigen::VectorXd dqLF(7), dqRF(7);
        dqLF << this->robot->dq(LeftHipRoll),
                this->robot->dq(LeftHipYaw),
                this->robot->dq(LeftHipPitch),
                this->robot->dq(LeftKneePitch),
                this->robot->dq(LeftShinPitch),
                this->robot->dq(LeftTarsusPitch),
                this->robot->dq(LeftFootPitch);
        dqRF << this->robot->dq(RightHipRoll),
                this->robot->dq(RightHipYaw),
                this->robot->dq(RightHipPitch),
                this->robot->dq(RightKneePitch),
                this->robot->dq(RightShinPitch),
                this->robot->dq(RightTarsusPitch),
                this->robot->dq(RightFootPitch);

        ca->GRF_desired << 0.0, 0.0, -this->weight * (1.0 - s) / 2.0,
                           0.0, 0.0, -this->weight * (1.0 + s) / 2.0;

        Matrix3d Rot = Eigen::EulerAnglesZYXd(0.0, this->robot->q(4), this->robot->q(3)).toRotationMatrix();
        ca->GRF_desired.block(0,0,3,1) << Rot * T_pelvis.rotation().transpose() * ca->GRF_desired.block(0,0,3,1);
        ca->GRF_desired.block(3,0,3,1) << Rot * T_pelvis.rotation().transpose() * ca->GRF_desired.block(3,0,3,1);

        // Compute
        ca->tau = ca->J_ff.transpose() * ca->GRF_desired;
        this->u << ca->tau(0), 0, ca->tau(2), ca->tau(3), ca->tau(6),
                   ca->tau(7), 0, ca->tau(9), ca->tau(10), ca->tau(13);

        return this->u;
    }
};


#endif // CASSIE_FF_CONGTROLLER_HPP
