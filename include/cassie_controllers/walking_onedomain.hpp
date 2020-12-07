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
        double V;
        VectorXd Fdes;
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
        double leg_angle_offset;

        bool qp_initialized;

        void init();
        void reset();
    } memory;

    /**
    * @brief Persistent controller configurations
    */
    struct Config {
        bool use_qp;
        int nDomain;
        double time_scale;
        double x_offset;
        double vx_offset;
        double vy_offset;
        double u_stancetoe_offset;
        double stoppable_velocity_threshold;
        bool use_contact_switching;
        VectorXd Kp;
        VectorXd Kd;
        VectorXd Kpy;
        VectorXd Kdy;
        double raibert_KpX;
        double raibert_KpY;
        double raibert_KdX;
        double raibert_KdY;
        double ddq_KpX;
        double ddq_KdX;
        double ddq_KiX;
        double ddq_KpY;
        double ddq_KdY;
        double ddq_KiY;
        double raibert_phaseThreshold;
        VectorXd torque_bounds;
        double left_stance_gravity_scale;
        double right_stance_gravity_scale;
        MatrixXd Be;
        double velocity_integrator_bleed_constant;

        double vdx_ub;
        double vdx_lb;
        double vdy_ub;
        double vdy_lb;

        struct GaitLibrary {
            struct StanceParameters{
                std::vector< std::vector<VectorXd> > amat_array;
                std::vector< std::vector<VectorXd> > apos_base_array;
                std::vector< std::vector<VectorXd> > avelocity_array;
                std::vector< std::vector<VectorXd> > addq_array;
                std::vector< std::vector<VectorXd> > af_array;

                void resize_vectors(unsigned long sz) {
                    amat_array.resize(sz);
                    apos_base_array.resize(sz);
                    avelocity_array.resize(sz);
                    addq_array.resize(sz);
                    af_array.resize(sz);
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

    control_utilities::LowPassFilter lpVdX    = control_utilities::LowPassFilter(0.001, 1.75);
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

    // Gait phasing variable
    PhaseVariable phase;

    // Swing leg tarsus ik solver
    RigidTarsusSolver tarsusSolver;

    // Private methods
    void nextDomain();
    void computeGuard(double x_offset, bool requestTransition);
    void computeActual(VectorXd &ya, VectorXd &dya, MatrixXd &Dya, MatrixXd &DLfya);
    void computeDesired(VectorXd &yd, VectorXd &dyd, VectorXd &d2yd);
    VectorXd getTorqueID();
    void updateTurning(double yawUpdate);
    void updateRaibert(double xVd, double yVd);
    void updateAccelerations(double xVd, double yVd);
    void computeLibrary();
};


#endif // WALKING_QP_HPP
