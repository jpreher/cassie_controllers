/**
    @author Jenna Reher (jreher@caltech.edu)
*/

#ifndef STANDING_CONTROL_QP_HPP
#define STANDING_CONTROL_QP_HPP

#include <cassie_description/cassie_model.hpp>
#include <control_utilities/filters.hpp>
#include <control_utilities/limits.hpp>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/EulerAngles>
#include <cassie_controllers/feedforward_controller.hpp>
#include <cassie_controllers/PD_controller.hpp>
#include <ros_utilities/timing.hpp>
#include <ros_utilities/ros_utilities.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <cassie_controllers/inverse_kinematics.hpp>
#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

class StandingControlQP {

public:
    StandingControlQP(ros::NodeHandle &nh, cassie_model::Cassie &robot);
    void update(Eigen::VectorXd &radio, Eigen::VectorXd &u);
    void reset();
    bool reconfigure();
    bool isReadyToTransition() {return this->memory.readyToTransition;}
    void getDebug(VectorXf &dbg);

private:
    /**
    * @brief Computations that can be reproduced given a Config and Memory object
    * Meant to only be valid during one computation frame. Anything meant to persist across time should be stored in Memory.
    */
    struct Cache {
        // Input
        VectorXd u;
        VectorXd qmd;
        VectorXd dqmd;
        MatrixXd Jik;

        // Outputs
        VectorXd ya;
        VectorXd dya;
        MatrixXd Dya;
        MatrixXd DLfya;
        VectorXd yd;
        VectorXd dyd;
        VectorXd d2yd;
        VectorXd eta;

        // Inverse kinematics helpers
        Eigen::Transform<double, 3, Affine> T_des_pelvis;
        Eigen::Transform<double, 3, Affine> T_des_leftFoot;
        Eigen::Transform<double, 3, Affine> T_des_rightFoot;
        double pitch_desired;
        Eigen::Vector4d leftTwist;
        Eigen::Vector4d rightTwist;
        Eigen::VectorXd dq_desired;
        Eigen::EulerAnglesZYXd euler;
        Eigen::VectorXd target_velocities;
        Eigen::VectorXd IK_solution;
        Eigen::MatrixXd pLF_actual;
        Eigen::MatrixXd pRF_actual;

        // QP cache allocation
        MatrixXd Jc;
        MatrixXd dJc;
        MatrixXd A_holonomics;
        VectorXd b_holonomics;
        MatrixXd Aeq_dynamics;
        VectorXd beq_dynamics;
        MatrixXd A_chatter;
        VectorXd b_chatter;
        MatrixXd A_friction;
        VectorXd b_lb_friction, b_ub_friction;
        MatrixXd A_y;
        VectorXd b_y;
        MatrixXd A_clf;
        MatrixXd A_reg;
        VectorXd b_reg;
        VectorXd qpsol;
        double V;
        VectorXd Fdes;
        double delta;

        // Struct methods
        void init();
        void reset();
    };
    Cache cache;

    /**
    * @brief Data meant to persist across data frames
    */
    struct Memory {
        int mode;
        bool readyToTransition;
        bool queueTransition;
        Eigen::VectorXd u_prev;
        Eigen::VectorXd IK_solution_prev;
        double spoolup;
        bool contact_initialized;

        bool crouchOverrideCompleted;
        bool crouchOverrideInitialized;
        ros_utilities::Timer crouchOverrideTimer = ros_utilities::Timer(true);

        Eigen::VectorXd dyd_last;

        bool qp_initialized;

        Eigen::Transform<double, 3, Affine> T_des_prev_leftFoot;
        Eigen::Transform<double, 3, Affine> T_des_prev_rightFoot;
        Eigen::Transform<double, 3, Affine> T_leftContact;
        Eigen::Transform<double, 3, Affine> T_rightContact;

        // Timer for keeping track of real-time control rates
        ros_utilities::Timer timer = ros_utilities::Timer(true);

        // Struct methods
        void init();
        void reset();
    };
    Memory memory;

    /**
    * @brief Persistent controller configurations
    */
    struct Config {
        bool use_qp;
        bool clf_use_Vdot_cost;
        bool clf_use_inequality;
        bool clf_use_task_pd;
        double control_rate;
        double height_lowpass_dt_cutoff;
        double lateral_lowpass_dt_cutoff;
        double pitch_lowpass_dt_cutoff;
        double x_lowpass_dt_cutoff;
        double x_com_offset;
        double ik_xtol;
        int ik_iter_limit;
        double height_lb;
        double height_ub;
        double lateral_lb;
        double lateral_ub;
        double pitch_lb;
        double pitch_ub;
        VectorXd torque_bounds;
        VectorXd Kp;
        VectorXd Kd;
        VectorXd Kpy;
        VectorXd Kdy;

        bool use_lateral_comp;
        double Kp_lateral_compensator;
        double Kd_lateral_compensator;

        // QP Terms
        int nQPIter;
        int ny;
        double ep;
        double gam;
        MatrixXd P;
        MatrixXd F;
        MatrixXd G;
        MatrixXd LFV_mat;
        MatrixXd Be;
        MatrixXd contact_pyramid;

        // QP Tuning
        VectorXd Pdiag;
        VectorXd Poffdiag;
        double reg_ddq;
        double reg_u;
        double reg_achilles;
        double reg_rigid;
        double reg_fx;
        double reg_fy;
        double reg_fz;
        double reg_muy;
        double reg_muz;
        double reg_clf_delta;
        double res_clf_ep;
        double clf_gam;

        double w_u_chatter;
        double w_outputs;
        double w_hol_achilles;
        double w_hol_fixed;
        double w_hol_fx;
        double w_hol_fy;
        double w_hol_fz;
        double w_hol_my;
        double w_hol_mz;
        double w_Vdot;

        // Parameter checker
        ros_utilities::ParamChecker paramChecker;

        void init();
        void reconfigure();
    } config;

    // SubControllers
    PD_Controller pd;

    // Inverse kinematics solver
    cassie_inverse_kinematics::StandingInverseKinematics IKfunction;

    // Pointer to the controlling nodehandle and related ROS things
    ros::NodeHandle *nh;
    ros::ServiceServer reconfigureService;

    // QP Solver for varying matrices
    SQProblem *qpsolver;

    // Robot model
    cassie_model::Cassie *robot;

    // Reconfigure callback for sub-node
    bool reconfigure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    // Various control methods
    void runInverseKinematics(MatrixXd &pLF, MatrixXd &pRF);
    VectorXd getTorqueQP();
    VectorXd getTorqueID();
    void computeDesired(VectorXd &radio);
    void computeActual();

    // Filters for smoothing joystick commands
    control_utilities::LowPassFilter heightFilter = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter pitchFilter = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter lateralFilter = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter leftXFilter = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter rightXFilter = control_utilities::LowPassFilter(NAN, NAN);

    // Core methods
    void setFootTransforms();
    void processRadio(Eigen::VectorXd &radio);
    int InverseKinematics(Eigen::VectorXd &x);
};

#endif // STANDING_CONTROL_QP_HPP
