/**
    @author J. Reher (jreher@caltech.edu)
*/

#ifndef WALKING_QP_HPP
#define WALKING_QP_HPP

#include <cassie_description/cassie_model.hpp>
#include <ros_utilities/ros_utilities.hpp>
#include <cassie_common_toolbox/PhaseVariable.hpp>
#include <cassie_common_toolbox/smoothing.hpp>
#include <std_srvs/Empty.h>
#include <ros_utilities/timing.hpp>
#include <cassie_controllers/feedforward_controller.hpp>
#include <qpOASES.hpp>
#include <control_utilities/filters.hpp>
#include <control_utilities/limits.hpp>
#include <cassie_estimation/rigidtarsus_solver.hpp>
#include <Eigen/Sparse>

USING_NAMESPACE_QPOASES

class Walking1DControl {
public:

    Walking1DControl(ros::NodeHandle &nh, cassie_model::Cassie &robot);
    void update(VectorXd &radio, VectorXd &u);
    void reset();
    bool reconfigure();
    bool isReadyToTransition() {return this->memory.readyToStop;}
    void getDebug(VectorXf &dbg);

private:

    /**
    * @brief Computations that can be reproduced given a Config and Memory object
    * Meant to only be valid during one computation frame. Anything meant to persist across time should be stored in Memory.
    */
    struct Cache {
        VectorXd ya;
        VectorXd dya;
        MatrixXd Dya;
        MatrixXd DLfya;
        VectorXd yd;
        VectorXd dyd;
        VectorXd d2yd;
        VectorXd ddqd;
        VectorXd pd;
        VectorXd vd;
        VectorXd Fd;
        VectorXd raibert_offset;
        VectorXd uff;
        VectorXd ddqtar;

        VectorXd qmd;
        VectorXd dqmd;


        // QP cache allocation
        MatrixXd Jc;
        MatrixXd dJc;
        MatrixXd Js;
        MatrixXd dJs;
        MatrixXd Jcs;
        MatrixXd dJcs;
        MatrixXd A_holonomics;
        VectorXd b_holonomics;
        MatrixXd Aeq_dynamics;
        VectorXd beq_dynamics;
        MatrixXd A_friction;
        VectorXd b_lb_friction, b_ub_friction;
        MatrixXd A_y;
        VectorXd b_y;
        MatrixXd A_clf;
        MatrixXd A_reg;
        VectorXd b_reg;
        MatrixXd Proj;
        VectorXd qpsol;
        double V;
        VectorXd Fdes;
        double delta;

        Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> Aconstr;
        VectorXd lbA;
        VectorXd ubA;

        Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> G;
        VectorXd g;
        Eigen::SparseMatrix<double> Gsparse;

        Eigen::EulerAnglesZYXd euler;
        Eigen::Transform<double, 3, Affine> T_des_pelvis;

        VectorXd u;

        void init();
        void reset();
    } cache;

    /**
    * @brief Data meant to persist across data frames
    */
    struct Memory {
        // Timer
        ros_utilities::Timer timer = ros_utilities::Timer(true);
        int iDomain;
        bool isInitialized;
        bool readyToStop;
        bool queueStop;
        Vector2d lastContact;
        VectorXd yd_last;
        VectorXd dyd_last;
        VectorXd raibert_offset_last;
        double   leg_angle_offset;
        double   leg_roll_offset;
        VectorXd u_prev;
        VectorXd vd_integral_error;

        MatrixXd paramCurrent;

        Vector2d raibert_deltaX;
        Vector2d raibert_deltaY;
        Vector2d lastStepVelocity;
        Vector2d velocityAllocator;
        int nVelocitySamplesThisStep;
        double lastTau;
        double s_feet; // (left) -1 <--> 1 (right)

        bool qp_initialized;

        void init();
        void reset();
    } memory;

    /**
    * @brief Persistent controller configurations
    */
    struct Config {
        bool isSim;
        bool use_qp;
        int nDomain;
        double time_scale;
        double vx_offset;
        double vy_offset;
        double leg_angle_offset_center;
        double u_stancetoe_offset;
        double swing_foot_offset;
        double stoppable_velocity_threshold;
        bool use_contact_switching;
        VectorXd Kp;
        VectorXd Kd;
        VectorXd Kpy;
        VectorXd Kdy;
        bool swing_angle_absolute;
        double raibert_KpX;
        double raibert_KpY;
        double raibert_KdX;
        double raibert_KdY;
        double raibert_KiX;
        double raibert_KiY;
        double ddq_KpX;
        double ddq_KdX;
        double ddq_KiX;
        double ddq_KpY;
        double ddq_KdY;
        double ddq_KiY;
        VectorXd torque_bounds;
        double left_stance_gravity_scale;
        double right_stance_gravity_scale;
        VectorXd Pdiag;
        VectorXd Poffdiag;
        double res_clf_ep;
        double clf_gam;
        double velocity_integrator_bleed_constant;

        double vdx_ub;
        double vdx_lb;
        double vdy_ub;
        double vdy_lb;

        double Kp_grf_leglength;
        double Kd_grf_leglength;
        double k1_grf;
        double k2_grf;
        double k3_grf;
        double k4_grf;
        double k_grf_offset;

        double force_z_scale;
        double force_sp_scale;

        bool clf_use_task_pd;
        bool clf_use_inequality;
        bool clf_use_Vdot_cost;

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
        VectorXd lb;
        VectorXd ub;

        // Large matrix static configurations
        MatrixXd Su;
        MatrixXd Sc;

        // QP Tuning
        double reg_ddq;
        VectorXd reg_base_ddq;
        VectorXd reg_stance_ddq;
        VectorXd reg_swing_ddq;
        VectorXd reg_stance_u;
        VectorXd reg_swing_u;
        double reg_rigid;
        double reg_fx;
        double reg_fy;
        double reg_fz;
        double reg_muy;
        double reg_muz;
        double reg_clf_delta;

        double w_outputs;
        double w_hol_fixed;
        double w_hol_fx;
        double w_hol_fy;
        double w_hol_fz;
        double w_hol_my;
        double w_hol_mz;
        double w_Vdot;

        struct GaitLibrary {
            struct StanceParameters{
                std::vector< std::vector<VectorXd> > amat_array;
                std::vector< std::vector<VectorXd> > apos_base_array;
                std::vector< std::vector<VectorXd> > avelocity_array;
                std::vector< std::vector<VectorXd> > addq_array;
                std::vector< std::vector<VectorXd> > af_array;
                std::vector< std::vector<VectorXd> > au_array;

                void resize_vectors(unsigned long sz) {
                    amat_array.resize(sz);
                    apos_base_array.resize(sz);
                    avelocity_array.resize(sz);
                    addq_array.resize(sz);
                    af_array.resize(sz);
                    au_array.resize(sz);
                }
            };
            StanceParameters left;
            StanceParameters right;
            int nDomain;
            Vector2d phaseRange;
            VectorXd vd_x;
            VectorXd vd_y;

        } gaitlib;

        // Parameter checker
        ros_utilities::ParamChecker paramChecker;

        // Methods
        void init();
        void reconfigure();

    } config;

    // Pointer to the controlling nodehandle and related ROS things
    ros::NodeHandle *nh;
    ros::ServiceServer reconfigureService;

    control_utilities::LowPassFilter lpVdX    = control_utilities::LowPassFilter(0.001, 0.75);
    control_utilities::LowPassFilter lpVdY    = control_utilities::LowPassFilter(0.001, 0.25);
    control_utilities::LowPassFilter lpVdTurn = control_utilities::LowPassFilter(0.001, 0.25);

    control_utilities::LowPassFilter lpVaX = control_utilities::LowPassFilter(0.001, 0.0001);
    control_utilities::LowPassFilter lpVaY = control_utilities::LowPassFilter(0.001, 0.0001);

    control_utilities::LowPassFilter lpVaXlastStep = control_utilities::LowPassFilter(0.001, 0.0001);
    control_utilities::LowPassFilter lpVaYlastStep = control_utilities::LowPassFilter(0.001, 0.0001);

    control_utilities::RateLimiter rateRaibertX    = control_utilities::RateLimiter(-75, 75); // (-0.2, 0.2);
    control_utilities::RateLimiter rateRaibertY    = control_utilities::RateLimiter(-75, 75); // (-0.2, 0.2);

    control_utilities::RateLimiter rateStepX    = control_utilities::RateLimiter(-0.15, 0.15);
    control_utilities::RateLimiter rateStepY    = control_utilities::RateLimiter(-0.10, 0.10);

    control_utilities::RateLimiter rateJoyX    = control_utilities::RateLimiter(-0.30, 0.30);
    control_utilities::RateLimiter rateJoyY    = control_utilities::RateLimiter(-0.30, 0.30);
    control_utilities::RateLimiter rateJoyTurn = control_utilities::RateLimiter(-0.15, 0.15);

    cassie_common_toolbox::MovingAverage stepPeriodAverager = cassie_common_toolbox::MovingAverage(400, 2); // Moving average for one step period (T=0.4s)

    // QP Solver for varying matrices
    SQProblem *qpsolver;

    // Reconfigure callback for sub-node
    bool reconfigure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    // Pointer to robot model
    cassie_model::Cassie *robot;

    // Feedforward controller
    feedforward_underactuated ff;

    // Gait phasing variable
    PhaseVariable phase;

    // Swing leg tarsus ik solver
    RigidTarsusSolver tarsusSolver;

    // Private methods
    void nextDomain();
    void computeGuard(bool requestTransition);
    void computeActual(VectorXd &ya, VectorXd &dya, MatrixXd &Dya, MatrixXd &DLfya);
    void computeDesired(VectorXd &yd, VectorXd &dyd, VectorXd &d2yd);
    VectorXd getTorqueQP();
    VectorXd getTorqueID();
    void updateTurning(double yawUpdate);
    void updateRaibert(double xVd, double yVd);
    void updateAccelerations(double xVd, double yVd);
    void updateForces(double xVd, double yVd);
    void computeLibrary();

    int SwingLegInverseKinematics(VectorXd &qmd, VectorXd &dqmd, VectorXd &ddqmd);
};


#endif // WALKING_QP_HPP
