/**
    @author Jenna Reher (jreher@caltech.edu)
*/

#include <cassie_controllers/walking_onedomain.hpp>
#include <cassie_common_toolbox/motion_transition.hpp>
#include <cassie_common_toolbox/RadioButtonMap.hpp>
#include <cassie_controllers/feedforward_controller.hpp>
#include <cassie_common_toolbox/bezier_tools.hpp>
#include <roslib_utilities/ros_package_utilities.hpp>
#include <control_utilities/limits.hpp>
#include <cassie_common_toolbox/linear_interpolation.hpp>

USING_NAMESPACE_QPOASES
using namespace control_utilities;

void Walking1DControl::Cache::init() {
    this->ya.resize(9);
    this->dya.resize(9);
    this->Dya.resize(9,44);
    this->DLfya.resize(9,44);
    this->yd.resize(9);
    this->dyd.resize(9);
    this->d2yd.resize(9);
    this->ddqd.resize(22);
    this->u.resize(10);
    this->uff.resize(10);
    this->qmd.resize(10);
    this->dqmd.resize(10);
    this->pd.resize(2);
    this->vd.resize(2);
    this->Fd.resize(7);
    this->raibert_offset.resize(3);
    this->ddqtar.resize(22);
    this->Fdes.resize(11);

    // Initialize matrices
    this->reset();
}

void Walking1DControl::Cache::reset() {
    this->ya.setZero();
    this->dya.setZero();
    this->Dya.setZero();
    this->DLfya.setZero();
    this->yd.setZero();
    this->dyd.setZero();
    this->d2yd.setZero();
    this->ddqd.setZero();
    this->dqmd.setZero();
    this->qmd.setZero();
    this->T_des_pelvis.setIdentity();
    this->euler = EulerAnglesZYXd(0.0, 0.0, 0.0);
    this->u.setZero();
    this->uff.setZero();
    this->pd.setZero();
    this->vd.setZero();
    this->Fd.setZero();
    this->raibert_offset.setZero();
    this->ddqtar.setZero();
    this->V = 0.;
    this->Fdes.setZero();
}

void Walking1DControl::Memory::init() {
    this->paramCurrent.resize(9,7);
    this->u_prev.resize(10);
    this->yd_last.resize(9);
    this->dyd_last.resize(9);
    this->raibert_offset_last.resize(3);
    this->vd_integral_error.resize(2);

    this->reset();
}

void Walking1DControl::Memory::reset() {
    this->iDomain = 0;
    this->s_feet = 0;
    this->isInitialized = false;
    this->readyToStop = false;
    this->queueStop = false;
    this->timer.restart();
    this->lastContact.setZero();
    this->raibert_deltaX.setZero();
    this->raibert_deltaY.setZero();
    this->lastStepVelocity.setZero();
    this->velocityAllocator.setZero();
    this->yd_last.setZero();
    this->dyd_last.setZero();
    this->raibert_offset_last.setZero();
    this->nVelocitySamplesThisStep = 0;
    this->lastTau = 0;
    this->qp_initialized = false;
    this->u_prev.setZero();
    this->vd_integral_error.setZero();
}

void Walking1DControl::Config::init() {
    this->Kp.resize(9); this->Kp.setZero();
    this->Kd.resize(9); this->Kd.setZero();
}

void Walking1DControl::Config::reconfigure() {
    this->paramChecker.checkAndUpdate("use_qp", this->use_qp);
    this->paramChecker.checkAndUpdate("nDomain", this->nDomain);
    this->paramChecker.checkAndUpdate("time_scale", this->time_scale);
    this->paramChecker.checkAndUpdate("x_offset", this->x_offset);
    this->paramChecker.checkAndUpdate("vx_offset", this->vx_offset);
    this->paramChecker.checkAndUpdate("vy_offset", this->vy_offset);
    this->paramChecker.checkAndUpdate("u_stancetoe_offset", this->u_stancetoe_offset);
    this->paramChecker.checkAndUpdate("stoppable_velocity_threshold", this->stoppable_velocity_threshold);
    this->paramChecker.checkAndUpdate("use_contact_switching", this->use_contact_switching);
    this->paramChecker.checkAndUpdateYaml("kp", this->Kp);//("qp/kp", this->Kp);
    this->paramChecker.checkAndUpdateYaml("kd", this->Kd);//("qp/kd", this->Kd);
    this->paramChecker.checkAndUpdateYaml("qp/kp", this->Kpy);//("qp/kp", this->Kp);
    this->paramChecker.checkAndUpdateYaml("qp/kd", this->Kdy);//("qp/kd", this->Kd);
    this->paramChecker.checkAndUpdate("left_stance_gravity_scale", this->left_stance_gravity_scale);
    this->paramChecker.checkAndUpdate("right_stance_gravity_scale", this->right_stance_gravity_scale);

    std::string raw_file_path, local_path;
    YAML::Node doc;
    this->paramChecker.checkAndUpdateYaml("gaitlib", local_path);
    raw_file_path = roslib_utilities::resolve_local_url(local_path).string();
    yaml_utilities::yaml_read_file(raw_file_path, doc);
    std::cout << "\t[ traj: " << raw_file_path << "]" << std::endl;

    // Get the size of the velocity gridding
    unsigned long nvx = doc["domain"].size();
    unsigned long nvy = doc["domain"][0].size();

    // Store values for velocities
    gaitlib.vd_x.resize(nvx);
    gaitlib.vd_y.resize(nvy);

    for (unsigned long i=0; i<nvx; i++) {
        doc["domain"][i][0]["vd_x"] >> gaitlib.vd_x[i];
    }
    for (unsigned long i=0; i<nvy; i++) {
        doc["domain"][0][i]["vd_y"] >> gaitlib.vd_y[i];
    }

    doc["domain"][0][0]["right"]["pposition"] >> gaitlib.phaseRange;

    gaitlib.left.resize_vectors(nvx);
    gaitlib.right.resize_vectors(nvx);

    VectorXd temp;
    for (unsigned long i=0; i<nvx; i++) {
        for (unsigned long j=0; j<nvy; j++) {
            // Right stance
            doc["domain"][i][j]["right"]["aposition"] >> temp;
            gaitlib.right.amat_array[i].push_back(temp);
            doc["domain"][i][j]["right"]["apbase"] >> temp;
            gaitlib.right.apos_base_array[i].push_back(temp);
            doc["domain"][i][j]["right"]["avelocity"] >> temp;
            gaitlib.right.avelocity_array[i].push_back(temp);
            doc["domain"][i][j]["right"]["addq"] >> temp;
            gaitlib.right.addq_array[i].push_back(temp);
            doc["domain"][i][j]["right"]["af"] >> temp;
            gaitlib.right.af_array[i].push_back(temp);

            // Left stance
            doc["domain"][i][j]["left"]["aposition"] >> temp;
            gaitlib.left.amat_array[i].push_back(temp);
            doc["domain"][i][j]["left"]["apbase"] >> temp;
            gaitlib.left.apos_base_array[i].push_back(temp);
            doc["domain"][i][j]["left"]["avelocity"] >> temp;
            gaitlib.left.avelocity_array[i].push_back(temp);
            doc["domain"][i][j]["left"]["addq"] >> temp;
            gaitlib.left.addq_array[i].push_back(temp);
            doc["domain"][i][j]["left"]["af"] >> temp;
            gaitlib.left.af_array[i].push_back(temp);
        }
    }

    // Robot torque
    this->Be.resize(22,10);
    this->Be.setZero();
    this->Be(LeftHipRoll,    0) = 25;
    this->Be(LeftHipYaw,     1) = 25;
    this->Be(LeftHipPitch,   2) = 16;
    this->Be(LeftKneePitch,  3) = 16;
    this->Be(LeftFootPitch,  4) = 50;
    this->Be(RightHipRoll,   5) = 25;
    this->Be(RightHipYaw,    6) = 25;
    this->Be(RightHipPitch,  7) = 16;
    this->Be(RightKneePitch, 8) = 16;
    this->Be(RightFootPitch, 9) = 50;

    // Safe torque bounds
    this->torque_bounds.resize(10);
    this->torque_bounds << 4.5, 4.5, 12.2, 12.2, 0.9,
            4.5, 4.5, 12.2, 12.2, 0.9;
}

Walking1DControl::Walking1DControl(ros::NodeHandle &nh, cassie_model::Cassie &robot) :
    nh(&nh), ff(robot) {
    // Main setup
    this->robot = &robot;

    // Parameter checker
    this->config.paramChecker.init(nh.getNamespace() + "/stepping");

    // All the storage
    this->cache.init();
    this->memory.init();
    this->config.init();

    // Service for calling reconfigures
    this->reconfigureService = nh.advertiseService("reconfigure_stepping", &Walking1DControl::reconfigure, this);
    this->reconfigure();
}

void Walking1DControl::reset() {
    this->cache.reset();
    this->memory.reset();
    this->lpVdX.reset();
    this->lpVdY.reset();
    this->lpVdTurn.reset();
    this->lpVaX.reset();
    this->lpVaY.reset();
    this->lpVaXlastStep.reset();
    this->lpVaYlastStep.reset();
    this->rateJoyX.reset(0.);
    this->rateJoyY.reset(0.);
    this->rateJoyTurn.reset(0.);
    this->rateRaibertX.reset(0.);
    this->rateRaibertY.reset(0.);
    this->rateStepX.reset(0.);
    this->rateStepY.reset(0.);
    this->stepPeriodAverager.reset();
}

bool Walking1DControl::reconfigure() {
    std::cout << "Polling rosparams under: " << this->config.paramChecker.node.getNamespace() << std::endl;

    this->config.reconfigure();
    this->phase.reconfigure(this->config.gaitlib.phaseRange, this->config.time_scale);

    this->config.paramChecker.checkAndUpdate("raibert/KpX", this->config.raibert_KpX);
    this->config.paramChecker.checkAndUpdate("raibert/KpY", this->config.raibert_KpY);
    this->config.paramChecker.checkAndUpdate("raibert/KdX", this->config.raibert_KdX);
    this->config.paramChecker.checkAndUpdate("raibert/KdY", this->config.raibert_KdY);
    this->config.paramChecker.checkAndUpdate("raibert/phaseThreshold", this->config.raibert_phaseThreshold);


    this->config.paramChecker.checkAndUpdate("grf_KpX", this->config.grf_KpX);
    this->config.paramChecker.checkAndUpdate("grf_KpY", this->config.grf_KpY);

    this->config.paramChecker.checkAndUpdate("ddq_KpX", this->config.ddq_KpX);
    this->config.paramChecker.checkAndUpdate("ddq_KdX", this->config.ddq_KdX);
    this->config.paramChecker.checkAndUpdate("ddq_KiX", this->config.ddq_KiX);
    this->config.paramChecker.checkAndUpdate("ddq_KpY", this->config.ddq_KpY);
    this->config.paramChecker.checkAndUpdate("ddq_KdY", this->config.ddq_KdY);
    this->config.paramChecker.checkAndUpdate("ddq_KiY", this->config.ddq_KiY);

    this->config.paramChecker.checkAndUpdate("vdx_ub", this->config.vdx_ub);
    this->config.paramChecker.checkAndUpdate("vdx_lb", this->config.vdx_lb);
    this->config.paramChecker.checkAndUpdate("vdy_ub", this->config.vdy_ub);
    this->config.paramChecker.checkAndUpdate("vdy_lb", this->config.vdy_lb);

    return true;
}

bool Walking1DControl::reconfigure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    this->reconfigure();
    return true;
}

void Walking1DControl::update(VectorXd &radio, VectorXd &u) {
    // Compute the phase and average velocity
    this->robot->dq(BasePosX) += this->config.vx_offset;
    //this->robot->dq(BasePosY) += 0.1 * radio(S2);
    if (this->memory.iDomain == 0) {
        this->robot->dq(BasePosY) += this->config.vy_offset;
    } else {
        this->robot->dq(BasePosY) -= this->config.vy_offset;
    }

    this->phase.update(this->memory.timer.elapsed());

    if (!this->memory.isInitialized) {
        // Motion transition on a first loop
        this->memory.timer.restart();
        this->phase.update(this->memory.timer.elapsed());

        // Robot will fall to left and slightly back
        this->memory.lastStepVelocity << -0.05, 0.15;
        this->lpVaX.update(0);
        this->lpVaY.update(0);
        this->lpVdX.update(0);
        this->lpVdY.update(0);
        this->lpVaXlastStep.update(this->memory.lastStepVelocity(0));
        this->lpVaYlastStep.update(this->memory.lastStepVelocity(1));

        // Compute initial parameters
        this->computeLibrary();
        this->computeActual(this->cache.ya, this->cache.dya, this->cache.Dya, this->cache.DLfya);
        this->memory.yd_last << this->cache.ya;
        this->memory.dyd_last << this->cache.dya;
        this->computeDesired(this->memory.yd_last, this->memory.dyd_last, this->cache.d2yd);

        // Initialized
        this->memory.isInitialized = true;
    }

    // Pull the radio outputs for movement
    bool requestTransition = (radio(SB) < 1.0);
    double x_offset = this->config.x_offset;
    double xVd = this->config.vdx_ub * radio(LV);
    if (radio(LV) < 0.0)
        xVd = -this->config.vdx_lb * radio(LV);
    double yVd = -this->config.vdy_ub * radio(LH);  // Left joystick = -1.0 but +y axis on robot
    double yawRate = -0.12 * radio(RH); // Directionality swap for yaw as well

    this->lpVdX.update(this->rateJoyX.update(0.001, xVd));
    this->lpVdY.update(this->rateJoyY.update(0.001, yVd));
    this->lpVdTurn.update(this->rateJoyTurn.update(0.001, yawRate));

    xVd = control_utilities::clamp(this->lpVdX.getValue(), -0.8, 1.9);
    yVd = control_utilities::clamp(this->lpVdY.getValue(), -0.4, 0.4);
    yawRate = this->lpVdTurn.getValue();

    // Get the current parameters from library
    this->computeLibrary();
    this->lpVaXlastStep.update(this->memory.lastStepVelocity(0));
    this->lpVaYlastStep.update(this->memory.lastStepVelocity(1));

    // Update estimated instantaneous velocity
    this->lpVaX.update(this->lpVaXlastStep.getValue() + this->robot->dq(BasePosX) - this->cache.vd(0));
    this->lpVaY.update(this->lpVaYlastStep.getValue() + this->robot->dq(BasePosY) - this->cache.vd(1));

    VectorXd va(2);
    va << this->lpVaX.getValue(), this->lpVaY.getValue();
    this->stepPeriodAverager.update(va);

    // Accumulate integrator error and decay quickly whenever the tracking error crosses zero
    double dt = 0.001;
    double ix = dt * (lpVaX.getValue() - xVd);
    double iy = dt * (lpVaY.getValue() - yVd);

    // Always decay
    this->memory.vd_integral_error(0) *= 0.9995;
    this->memory.vd_integral_error(1) *= 0.9995;
    this->memory.vd_integral_error(0) += ix;
    this->memory.vd_integral_error(1) += iy;

    // Get the outputs and update kinematics
    this->robot->kinematics.update(this->robot->model, this->robot->q, this->robot->dq);
    this->computeActual(this->cache.ya, this->cache.dya, this->cache.Dya, this->cache.DLfya);
    this->computeDesired(this->cache.yd, this->cache.dyd, this->cache.d2yd);

    // Augment polynomials for foot placement and dynamics feedback
    this->cache.yd(5) += 0.037 + 0.05 * radio(S1); // Swing leg angle offset
    this->updateTurning(yawRate);
    this->updateRaibert(xVd, yVd);
    this->updateAccelerations(xVd, yVd);

    // Compute controller
    if ( this->memory.isInitialized && this->config.use_qp ) {
        ROS_WARN("THE QP CONTROLLER IS NOT IMPLEMENTED IN THIS VERSION!");
        this->cache.u.setZero();
    } else {
        this->cache.u = this->getTorqueID(xVd, yVd);
    }

    int ind = 0;
    for (int i=0; i<robot->iRotorMap.size(); ++i) {
        ind = robot->iRotorMap(i);
        this->cache.qmd(i) = this->robot->q(ind);
    }

    // Update guard
    this->computeGuard(x_offset, requestTransition);

    // Return
    this->memory.lastTau = this->phase.tau;
    u = this->cache.u;

    // Update velocity average for step
    this->memory.velocityAllocator(0) += this->lpVaX.getValue();
    this->memory.velocityAllocator(1) += this->lpVaY.getValue();
    this->memory.nVelocitySamplesThisStep += 1;
}

VectorXd Walking1DControl::getTorqueID(double xVd, double yVd) {
    // https://journals.sagepub.com/doi/pdf/10.1177/0278364910388677
    // Learning, planning, and control for quadruped locomotion over challenging terrain
    // Controller based on ^^
    VectorXi iActive(9);
    MatrixXd J(9,10);
    if (this->memory.iDomain == 0) {
        SymFunction::J_yaRightStance(J, this->robot->q);
    } else {
        SymFunction::J_yaLeftStance(J, this->robot->q);
    }

    // Get output PD tracking torques
    VectorXd upd(10), u(10);
    upd.setZero(); u.setZero();

    VectorXd aNS(16), aST(16);
    aNS << 1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.;
    aST << 0.,0.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.;

    double cST, cNS;
    cST = bezier_tools::bezier(aST, this->phase.tau);
    cNS = bezier_tools::bezier(aNS, this->phase.tau);

    VectorXd tau(10);
    tau = this->config.Kp.cwiseProduct(this->cache.ya - this->cache.yd) + this->config.Kd.cwiseProduct(this->cache.dya - this->cache.dyd);
    upd = - J.completeOrthogonalDecomposition().solve(tau);

    // Convert to motor side torque
    VectorXd torqueDownScale(10);
    torqueDownScale << 1./25., 1./25., 1./16., 1./16., 1./50.,
                       1./25., 1./25., 1./16., 1./16., 1./50.;
    u << torqueDownScale.cwiseProduct(upd);

    // Compute model dynamics
    MatrixXd Ge(22,1); Ge.setZero();
    SymFunction::Ge_cassie_v4(Ge, this->robot->q);
    this->robot->dynamics.calcHandC(this->robot->model, this->robot->q, this->robot->dq);

    // Compute the robot constraints
    MatrixXd Jc(11,22);
    double gravScale = 0.;
    if (this->memory.iDomain == 0) {
        gravScale = this->config.right_stance_gravity_scale;

        Jc << this->robot->kinematics.cache.J_achilles,
            this->robot->kinematics.cache.J_poseRightConstraint,
            this->robot->kinematics.cache.J_rigid;
    } else {
        gravScale = this->config.left_stance_gravity_scale;

        Jc << this->robot->kinematics.cache.J_achilles,
            this->robot->kinematics.cache.J_poseLeftConstraint,
            this->robot->kinematics.cache.J_rigid;
    }
    this->robot->dynamics.C = this->robot->dynamics.C - (1. - gravScale) * Ge;

    // Compute feedforward
    // https://journals.sagepub.com/doi/pdf/10.1177/0278364913519834
    // https://journals.sagepub.com/doi/pdf/10.1177/0278364910388677
    // https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5509646
    // Selection matrices for constrained/unconstrained
    VectorXd ff(10);
    ff.setZero();

    MatrixXd Su(11,22); Su.setZero();
    MatrixXd Sc(11,22); Sc.setZero();
    Sc.block(0,0,11,11) << MatrixXd::Identity(11,11);
    Su.block(0,11,11,11) << MatrixXd::Identity(11,11);

    // QR decomposition
    HouseholderQR<MatrixXd> qr(Jc.transpose());
    MatrixXd Q, RO, R; Q.setZero(); R.setZero();
    RO = qr.matrixQR().triangularView<Upper>();
    Q = qr.householderQ();
    R = RO.block(0,0,11,11);

    // Feedforward
    VectorXd H(22), ddqfb(22);
    H = this->robot->dynamics.H*(this->cache.ddqd) + this->robot->dynamics.C;
    this->cache.uff = (Su*Q.transpose()*this->config.Be).completeOrthogonalDecomposition().solve(Su) * Q.transpose() * H;
    u += this->cache.uff;

    // Compute GRF and force control
    VectorXd forces(11);
    forces = R.inverse() * Sc * Q.transpose() * (H - this->config.Be*(u+ff));
    this->cache.Fdes << forces;

    // Store the resulting grf
    this->cache.Fdes.setZero();
    this->cache.Fdes.segment(0,2) = forces.segment(0,2); // Pushrods
    this->cache.Fdes.segment(7,4) = forces.segment(7,4); // Pseudo-fixed springs
    if (this->memory.iDomain == 0) {
        this->cache.Fdes.segment(2,5) = forces.segment(2,5); // Left foot
    } else {
        this->cache.Fdes.segment(2,5) = forces.segment(2,5); // Right foot
    }

    // Cap saggital moment on foot from torque
    double pax, pay;
    VectorXd pose_sf(5);
    if (this->memory.iDomain == 0) {
        VectorWrap pose_sf_(pose_sf);
        SymFunction::p_rightSole_constraint(pose_sf_, this->robot->q);
    } else {
        VectorWrap pose_sf_(pose_sf);
        SymFunction::p_leftSole_constraint(pose_sf_, this->robot->q);
    }
    pax = - (pose_sf(0) + this->memory.raibert_offset_last(0));
    pay = - (pose_sf(1) + this->memory.raibert_offset_last(1));

    double moment_scale = 1.0/50.0;
    double foot_length = 0.14;
    double pitch_constr = moment_scale * forces(4) * foot_length / 2;

    if (this->memory.iDomain == 0) {
        u(9) += this->config.u_stancetoe_offset;
        if (u(9) > pitch_constr)
            u(9) = pitch_constr;
        if (u(9) < -pitch_constr)
            u(9) = -pitch_constr;
    } else {
        u(4) += this->config.u_stancetoe_offset;
        if (u(4) > pitch_constr)
            u(4) = pitch_constr;
        if (u(4) < -pitch_constr)
            u(4) = -pitch_constr;
    }

    // Return
    u += ff;
    VectorXd torqueScale(10);
    torqueScale << 25., 25., 16., 16., 50., 25., 25., 16., 16., 50.;
    this->cache.u << torqueScale.cwiseProduct(u);
    this->cache.uff = torqueScale.cwiseProduct(this->cache.uff);
    return this->cache.u;
}

void Walking1DControl::updateTurning(double yawUpdate) {
    VectorXd aNS(5), aST(5);
    aST << 1.,1.,0.,0.,0.;
    aNS << 0.,0.,1.,1.,1.;

    double sST, sNS, dsST, dsNS, d2ST, d2NS;
    sST = bezier_tools::bezier(aST, this->phase.tau);
    sNS = bezier_tools::bezier(aNS, this->phase.tau);
    dsST = bezier_tools::dbezier(aST, this->phase.tau) * this->phase.dtau;
    dsNS = bezier_tools::dbezier(aNS, this->phase.tau) * this->phase.dtau;
    d2ST = bezier_tools::d2bezier(aST, this->phase.tau) * pow(this->phase.dtau,2);
    d2NS = bezier_tools::d2bezier(aNS, this->phase.tau) * pow(this->phase.dtau,2);

    // Turning
    this->cache.yd(2) += sST * yawUpdate;
    this->cache.yd(7) += sNS * yawUpdate;
    this->cache.dyd(2) += dsST * yawUpdate;
    this->cache.dyd(7) += dsNS * yawUpdate;

    // Assign to feedforward terms
    if (this->memory.iDomain == 0) {
        this->cache.ddqd(RightHipYaw) += d2ST * yawUpdate;
        this->cache.ddqd(LeftHipYaw)  += d2NS * yawUpdate;
    } else {
        this->cache.ddqd(LeftHipYaw)  += d2ST * yawUpdate;
        this->cache.ddqd(RightHipYaw) += d2NS * yawUpdate;
    }
}

void Walking1DControl::updateRaibert(double xVd, double yVd) {
    double vax, vay, vprevx, vprevy;
    VectorXd va_avg = this->stepPeriodAverager.getValue();
    vax = this->lpVaX.getValue();
    vay = this->lpVaY.getValue();
    vprevx = this->lpVaXlastStep.getValue();
    vprevy = this->lpVaYlastStep.getValue();

    // Make swing leg respond to pitch/roll error
    // Leg pitch
    this->cache.yd(5)  += (this->cache.ya(1) - this->cache.yd(1));
    this->cache.dyd(5) += (this->cache.dya(1) - this->cache.dyd(1));
    // Hip roll
    this->cache.yd(6)  -= (this->cache.ya(0) - this->cache.yd(0));
    this->cache.dyd(6) -= (this->cache.dya(0) - this->cache.dyd(0));

    // Raibert
    double xo, yo;
    xo = this->config.raibert_KpX * (vax - xVd)
         + this->config.raibert_KdX * (vax - vprevx);
    xo = control_utilities::clamp(xo, -0.6, 0.6);
    xo = this->rateRaibertX.update(0.001, xo);

    yo = this->config.raibert_KpY * (vay - yVd*0.6)
         + this->config.raibert_KdY * (vay - vprevy);
    yo = this->rateRaibertY.update(0.001, yo);

    // Compute current foot positions
    MatrixXd plf(3,1), prf(3,1), pcommand(3,1);
    pcommand << Vector3d::Zero();
    SymFunction::position_leftFoot(plf, this->robot->q);
    SymFunction::position_rightFoot(prf, this->robot->q);

    // Compute leg angles and lengths from foot placement
    double roll_ = this->cache.yd(0);
    double pitch_ = this->cache.yd(1);
    double lp_ = this->cache.yd(5);
    double lr_ = this->cache.yd(6);
    double ll_ = this->cache.yd(4);

    // Nominal foot position
    double px_ =  - ll_ * sin( lp_ + pitch_ );
    double py_ =    ll_ * sin( lr_ + roll_ );
    double pz_ = sqrt(pow(ll_,2) - pow(px_,2) - pow(py_,2));

    // Collision check and saturation
    Vector3d footDiff;
    if (this->memory.iDomain == 0) {
        plf(0) = px_ + xo;
        plf(1) = py_ + 0.1305 + yo;
    } else {
        prf(0) = px_ + xo;
        prf(1) = py_ - 0.1305 + yo;
    }
    footDiff << plf - prf;
    double boundWidth = 0.08;
    if (footDiff(1) < boundWidth ) {
        // Get the yo at the saturation boundary
        if (this->memory.iDomain == 0)
            yo = boundWidth + prf(1) - 0.1305 - py_;
        else
            yo = plf(1) + 0.1305 - boundWidth - py_;
        // ROS_WARN("Foot bounding box violation!");
    }

    // New output values
    double ll = sqrt(pow(px_ + xo, 2) + pow(py_ + yo, 2) + pow(pz_, 2));
    double lp = asin( - (px_ + xo) / ll ) - pitch_;
    double lr = asin(   (py_ + yo) / ll ) - roll_;

    // Compute a Bezier polynomial for the new movement
    double lo = (ll - ll_);
    VectorXd newOutputs(3), d_newOutputs(3), d2_newOutputs(3);
    MatrixXd aNewOutputs(3,5);
    aNewOutputs <<
            0.,lo,lo,lo,lo,
            0.,(lp - lp_),(lp - lp_),(lp - lp_),(lp - lp_),
            0.,(lr - lr_),(lr - lr_),(lr - lr_),(lr - lr_);

    bezier_tools::bezier(aNewOutputs, this->phase.tau, newOutputs);
    bezier_tools::dbezier(aNewOutputs, this->phase.tau, d_newOutputs);
    bezier_tools::d2bezier(aNewOutputs, this->phase.tau, d2_newOutputs);
    d_newOutputs = d_newOutputs * this->phase.dtau;
    d2_newOutputs = d2_newOutputs * pow(this->phase.dtau, 2);

    // Assign to outputs
     this->cache.yd(4)  += newOutputs(0);
     this->cache.dyd(4) += d_newOutputs(0);
     this->cache.yd(5)  += newOutputs(1);
     this->cache.dyd(5) += d_newOutputs(1);
     this->cache.yd(6)  += newOutputs(2);
     this->cache.dyd(6) += d_newOutputs(2);

    if (this->memory.iDomain == 0) {
        this->cache.ddqd(LeftKneePitch) += d2_newOutputs(0) * (1./this->cache.Dya(4,LeftKneePitch));
        this->cache.ddqd(LeftKneePitch) += d2_newOutputs(1) * (1./this->cache.Dya(5,LeftKneePitch));
        this->cache.ddqd(LeftHipPitch) += d2_newOutputs(1) * (1./this->cache.Dya(5,LeftHipPitch));
        this->cache.ddqd(LeftHipRoll) += d2_newOutputs(2) * (1./this->cache.Dya(6,LeftHipRoll));
    } else {
        this->cache.ddqd(RightKneePitch) += d2_newOutputs(0) * (1./this->cache.Dya(4,RightKneePitch));
        this->cache.ddqd(RightKneePitch) += d2_newOutputs(1) * (1./this->cache.Dya(5,RightKneePitch));
        this->cache.ddqd(RightHipPitch) += d2_newOutputs(1) * (1./this->cache.Dya(5,RightHipPitch));
        this->cache.ddqd(RightHipRoll) += d2_newOutputs(2) * (1./this->cache.Dya(6,RightHipRoll));
    }

    // Store
    this->cache.raibert_offset << newOutputs(0), xo, yo;
}

void Walking1DControl::updateAccelerations(double xVd, double yVd) {
    // Compute ddq PD control
    VectorXd va_avg = this->stepPeriodAverager.getValue();
    double pax, pay;
    VectorXd pose_sf(5);
    if (this->memory.iDomain == 0) {
        VectorWrap pose_sf_(pose_sf);
        SymFunction::p_rightSole_constraint(pose_sf_, this->robot->q);
    } else {
        VectorWrap pose_sf_(pose_sf);
        SymFunction::p_leftSole_constraint(pose_sf_, this->robot->q);
    }
    pax = - (pose_sf(0) + this->memory.raibert_offset_last(0));
    pay = - (pose_sf(1) + this->memory.raibert_offset_last(1));

    this->cache.ddqd(0) += - (this->config.ddq_KpX * ( pax - this->cache.pd(0) ) + this->config.ddq_KdX * (this->lpVaXlastStep.getValue() - xVd) + this->config.ddq_KiX * this->memory.vd_integral_error(0));
    this->cache.ddqd(1) += - (this->config.ddq_KpY * ( pay - this->cache.pd(1) ) + this->config.ddq_KdY * (this->lpVaYlastStep.getValue() - yVd) + this->config.ddq_KiY * this->memory.vd_integral_error(1));
}

void Walking1DControl::nextDomain() {
    // Increment the domain indexing
    this->memory.iDomain += 1;
    if ( this->memory.iDomain >= this->config.nDomain )
        this->memory.iDomain = 0;
    this->memory.timer.restart();

    // Update Raibert controller things
    this->memory.lastStepVelocity = this->memory.velocityAllocator / (double)this->memory.nVelocitySamplesThisStep;
    // this->memory.lastStepVelocity(0) = this->rateStepX.update(0.4, this->memory.lastStepVelocity(0));
    // this->memory.lastStepVelocity(1) = this->rateStepY.update(0.4, this->memory.lastStepVelocity(1));
    this->memory.velocityAllocator.setZero();
    this->memory.nVelocitySamplesThisStep = 0;

    // Store terminal raibert offset
    this->memory.raibert_offset_last << this->cache.raibert_offset;
    //    std::cout << this->memory.raibert_offset_last.transpose() << std::endl;

    this->rateRaibertX.reset(0.);
    this->rateRaibertY.reset(0.);
}

void Walking1DControl::computeActual(VectorXd &ya, VectorXd &dya, MatrixXd &Dya, MatrixXd &DLfya) {
    if (this->memory.iDomain == 0) {
        VectorWrap ya_(ya), dya_(dya);
        SymFunction::yaRightStance(ya_, this->robot->q);
        SymFunction::dyaRightStance(dya_, this->robot->q, this->robot->dq);
        SymFunction::Dya_RightStanceActual(Dya, this->robot->q);
        SymFunction::DLfya_RightStanceActual(DLfya, this->robot->q, this->robot->dq);
    } else {
        VectorWrap ya_(ya), dya_(dya);
        SymFunction::yaLeftStance(ya_, this->robot->q);
        SymFunction::dyaLeftStance(dya_, this->robot->q, this->robot->dq);
        SymFunction::Dya_LeftStanceActual(Dya, this->robot->q);
        SymFunction::DLfya_LeftStanceActual(DLfya, this->robot->q, this->robot->dq);
    }
}

void Walking1DControl::computeDesired(VectorXd &yd, VectorXd &dyd, VectorXd &d2yd) {
    // Update with smoothing from last domain
    bezier_tools::bezier(this->memory.paramCurrent, this->phase.tau, yd);
    bezier_tools::dbezier(this->memory.paramCurrent, this->phase.tau, dyd);
    bezier_tools::dbezier(this->memory.paramCurrent, this->phase.tau, d2yd);
    dyd = dyd * this->phase.dtau;
    d2yd = d2yd * pow(this->phase.dtau,2);

    // Move toe up a little
    yd(8) -= 0.05;

    // Use the impact leg extension for part of the following step
    yd(3) += (1.0 - this->phase.tau) * this->memory.raibert_offset_last(0);
}

void Walking1DControl::computeGuard(double x_offset, bool requestTransition) {
    // Check if we are stoppable
    bool stoppable = ((fabs(this->memory.velocityAllocator(0)) + fabs(this->memory.velocityAllocator(1))) / this->memory.nVelocitySamplesThisStep < this->config.stoppable_velocity_threshold);
    bool guard = (this->phase.tau >= 1.0);

    if (this->memory.iDomain == 0) {
        if (this->phase.tau >= 0.75)
            guard = guard || ((this->robot->leftContact > 0.25) && this->config.use_contact_switching);
    } else {
        if (this->phase.tau >= 0.75)
            guard = guard || ((this->robot->rightContact > 0.25) && this->config.use_contact_switching);
    }

    // If guard was triggered, handle domain switch
    this->memory.readyToStop = false;
    if (guard) {
        if (this->memory.queueStop)
            this->memory.readyToStop = true;

        MatrixXd lastParam(this->memory.paramCurrent.rows(), this->memory.paramCurrent.cols());
        lastParam << this->memory.paramCurrent;

        // Compute terminal outputs
        this->computeDesired(this->cache.yd, this->cache.dyd, this->cache.d2yd);

        // Increment
        this->nextDomain();

        // Update terminal outputs
        this->computeActual(this->cache.ya, this->cache.dya, this->cache.Dya, this->cache.DLfya);
        double swingRoll, dSwingRoll;
        if (this->memory.iDomain == 0) {
            swingRoll = this->robot->q(LeftHipRoll);
            dSwingRoll = this->robot->dq(LeftHipRoll);
        } else {
            swingRoll = this->robot->q(RightHipRoll);
            dSwingRoll = this->robot->dq(RightHipRoll);
        }

        this->memory.yd_last <<
                this->cache.yd(0),
                this->cache.yd(1),
                this->cache.yd(7),
                this->cache.yd(4),
                this->cache.yd(3),
                this->cache.ya(5),
                swingRoll,
                this->cache.yd(2),
                0.;
        this->memory.dyd_last <<
                this->cache.dyd(0),
                this->cache.dyd(1),
                this->cache.dyd(7),
                this->cache.dyd(4),
                this->cache.dyd(3),
                this->cache.dya(5),
                dSwingRoll,
                this->cache.dyd(2),
                0.;

        // If the new domain is right stance and stop was requested, then queue in the right to stance parameters
        if ((this->memory.iDomain == 0) && (requestTransition) && (stoppable)) {
            this->memory.queueStop = true;
            // NOT IMPLEMENTED!
            //this->memory.paramCurrent << this->config.paramRightToStand;
        }
    }
}

void Walking1DControl::computeLibrary() {
    VectorXd va = this->stepPeriodAverager.getValue();

    // Get the desired
    double v_step_x = control_utilities::clamp(this->lpVaXlastStep.getValue(), -0.8, 1.9);
    double v_step_y = control_utilities::clamp(this->lpVaYlastStep.getValue(), -0.5, 0.5);

    // Interpolate the gait
    VectorXd tempa(this->config.gaitlib.right.amat_array[0][0].cols());
    VectorXd tempddq(this->config.gaitlib.right.avelocity_array[0][0].cols());
    VectorXd temppd(this->config.gaitlib.right.addq_array[0][0].cols());
    VectorXd tempvd(this->config.gaitlib.right.addq_array[0][0].cols());
    VectorXd tempFd(this->config.gaitlib.right.af_array[0][0].cols());
    if (this->memory.iDomain == 0) {
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.right.amat_array,
                                               v_step_x, v_step_y, tempa);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.right.addq_array,
                                               v_step_x, v_step_y, tempddq);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.right.apos_base_array,
                                               v_step_x, v_step_y, temppd);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.right.avelocity_array,
                                               v_step_x, v_step_y, tempvd);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.right.af_array,
                                               v_step_x, v_step_y, tempFd);
    }
    else {
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.left.amat_array,
                                               v_step_x, v_step_y, tempa);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.left.addq_array,
                                               v_step_x, v_step_y, tempddq);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.left.apos_base_array,
                                               v_step_x, v_step_y, temppd);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.left.avelocity_array,
                                               v_step_x, v_step_y, tempvd);
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.left.af_array,
                                               v_step_x, v_step_y, tempFd);
    }

    // Reshape parameters
    Map<MatrixXd> tempM(tempa.data(), 9,7);
    Map<MatrixXd> tempDDQ(tempddq.data(), 22,7);
    Map<MatrixXd> tempP(temppd.data(), 2,7);
    Map<MatrixXd> tempV(tempvd.data(), 2,7);
    Map<MatrixXd> tempF(tempFd.data(), 7,7);

    this->memory.paramCurrent << tempM;
    bezier_tools::bezier(tempP, this->phase.tau, this->cache.pd);
    bezier_tools::bezier(tempV, this->phase.tau, this->cache.vd);
    bezier_tools::bezier(tempDDQ, this->phase.tau, this->cache.ddqd);
    bezier_tools::bezier(tempF, this->phase.tau, this->cache.Fd);
}

void Walking1DControl::getDebug(VectorXd &dbg) {
    dbg << ros::Time::now().toSec(), // 1
            this->phase.tau, // 1
            this->phase.dtau, // 1
            this->cache.ya, // 9
            this->cache.dya, // 9
            this->cache.yd, // 9
            this->cache.dyd, // 9
            this->cache.V, // 1
            this->cache.u, // 10
            this->cache.Fdes, // 11
            0, // 1
            this->cache.vd(0), this->cache.vd(1), // 2
            this->robot->dq(0), this->robot->dq(1), // 2
            this->lpVaXlastStep.getValue(), this->lpVaYlastStep.getValue(), //2
            this->cache.raibert_offset, // 3
            this->cache.uff; // 10
    // ndbg = 76// + 44
}
