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

    // Size all QP matrices for speeeeeed
    this->Jc.resize(4,22);
    this->dJc.resize(4,22);
    this->Js.resize(7,22);
    this->dJs.resize(7,22);
    this->Jcs.resize(11,22);
    this->dJcs.resize(11,22);
    this->A_holonomics.resize(7,40);
    this->b_holonomics.resize(7);
    this->Aeq_dynamics.resize(22, 40);
    this->beq_dynamics.resize(22);
    this->A_friction.resize(9,40);
    this->b_lb_friction.resize(9);
    this->b_ub_friction.resize(9);
    this->A_y.resize(9,40);
    this->b_y.resize(9);
    this->A_clf.resize(1,40);
    this->A_reg.resize(40,40);
    this->b_reg.resize(40);
    this->qpsol.resize(40);
    this->Fdes.resize(11);
    this->g.resize(40);
    this->G.resize(40,40);
    this->Gsparse.resize(40,40);
    this->Aconstr.resize(32,40);
    this->lbA.resize(32);
    this->ubA.resize(32);
    this->Proj.resize(22,22);

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

    this->Jc.setZero();
    this->dJc.setZero();
    this->Js.setZero();
    this->dJs.setZero();
    this->Jcs.setZero();
    this->dJcs.setZero();
    this->A_holonomics.setZero();
    this->b_holonomics.setZero();
    this->Aeq_dynamics.setZero();
    this->beq_dynamics.setZero();
    this->A_friction.setZero();
    this->b_lb_friction.setZero();
    this->b_ub_friction.setZero();
    this->A_y.setZero();
    this->b_y.setZero();
    this->A_clf.setZero();
    this->A_reg.setZero();
    this->A_reg << MatrixXd::Identity(40,40);
    this->b_reg.setZero();
    this->qpsol.setZero();
    this->V = 0.;
    this->Fdes.setZero();
    this->delta = 0.;
    this->Gsparse.setZero();
    this->g.setZero();
    this->G.setZero();
    this->Aconstr.setZero();
    this->lbA.setZero();
    this->ubA.setZero();
    this->Proj.setZero();
}

void Walking1DControl::Memory::init() {
    this->paramCurrent.resize(9,7);
    this->u_prev.resize(10);
    this->yd_last.resize(9);
    this->dyd_last.resize(9);
    this->leg_angle_offset = 0.;
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
    this->leg_roll_offset = 0;
    this->leg_angle_offset = 0;
}

void Walking1DControl::Config::init() {
    this->Kp.resize(9); this->Kp.setZero();
    this->Kd.resize(9); this->Kd.setZero();
    this->Su.resize(11,22);
    this->Sc.resize(11,22);
}

void Walking1DControl::Config::reconfigure() {
    this->isSim = false;
    ros::param::get("/cassie/is_simulation", this->isSim);
    this->paramChecker.checkAndUpdate("use_qp", this->use_qp);
    this->paramChecker.checkAndUpdate("nDomain", this->nDomain);
    this->paramChecker.checkAndUpdate("time_scale", this->time_scale);
    this->paramChecker.checkAndUpdate("vx_offset", this->vx_offset);
    this->paramChecker.checkAndUpdate("vy_offset", this->vy_offset);
    this->paramChecker.checkAndUpdate("swing_foot_offset", this->swing_foot_offset);
    this->paramChecker.checkAndUpdate("leg_angle_offset_center", this->leg_angle_offset_center);
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
    //doc["phaseRange"] >> this->gaitlib.phaseRange;
    //doc["vd"] >> this->gaitlib.vd;
    //doc["amat"] >> this->gaitlib.amat;

    //std::cout << "Library step frequency: " << this->gaitlib.phaseRange[1] << "s, speed range: " << this->gaitlib.vd[0] << " -> " << this->gaitlib.vd[this->gaitlib.vd.size()-1] << std::endl;

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
            doc["domain"][i][j]["right"]["au"] >> temp;
            gaitlib.right.au_array[i].push_back(temp);

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
            doc["domain"][i][j]["left"]["au"] >> temp;
            gaitlib.left.au_array[i].push_back(temp);
        }
    }

    // Precomputed CLF terms and configurable gains
    this->paramChecker.checkAndUpdate("qp/res_clf_ep", this->res_clf_ep);
    this->paramChecker.checkAndUpdate("qp/clf_gam", this->clf_gam);
    this->ny  = 9;
    this->ep  = this->res_clf_ep;
    this->gam = this->clf_gam / this->ep; // min(eig(obj.Q))/ max(eig(obj.P)) / obj.ep; // precomputed in Matlab

    // CLF terms for output dynamics
    this->F.resize(2*ny,2*ny);
    this->F.setZero();
    this->F.topRightCorner(ny,ny) = MatrixXd::Identity(ny,ny);
    this->G.resize(2*ny,ny);
    this->G.setZero();
    this->G.bottomRows(ny) = MatrixXd::Identity(ny,ny);

    // Exponential CLF and CARE solution
    this->paramChecker.checkAndUpdateYaml("qp/Pdiag", this->Pdiag);
    this->paramChecker.checkAndUpdateYaml("qp/Poffdiag", this->Poffdiag);
    this->P.resize(2*ny,2*ny); this->P.setZero();
    this->P.diagonal() << this->Pdiag;
    this->P.topRightCorner(ny,ny).diagonal() << this->Poffdiag;
    this->P.bottomLeftCorner(ny,ny).diagonal() << this->Poffdiag;

    MatrixXd Iep(2*ny,2*ny);
    Iep.setZero();
    Iep.topLeftCorner(ny,ny) << MatrixXd::Identity(ny,ny) * (1.0/this->ep);
    Iep.bottomRightCorner(ny,ny) << MatrixXd::Identity(ny,ny);
    this->P = Iep.transpose() * this->P * Iep;

    // Derivative of linear output dynamics
    this->LFV_mat = this->F.transpose() * this->P + this->P * this->F;

    // Robot torque
    this->Be.resize(22,10);
    this->Be.setZero();
    this->Be(LeftHipRoll,    0) = 1.;
    this->Be(LeftHipYaw,     1) = 1.;
    this->Be(LeftHipPitch,   2) = 1.;
    this->Be(LeftKneePitch,  3) = 1.;
    this->Be(LeftFootPitch,  4) = 1.;
    this->Be(RightHipRoll,   5) = 1.;
    this->Be(RightHipYaw,    6) = 1.;
    this->Be(RightHipPitch,  7) = 1.;
    this->Be(RightKneePitch, 8) = 1.;
    this->Be(RightFootPitch, 9) = 1.;

    // Orthogonal projection matrices
    this->Su.setZero();
    Su.block(0,11,11,11) << MatrixXd::Identity(11,11);
    this->Sc.setZero();
    Sc.block(0,0,11,11) << MatrixXd::Identity(11,11);

    // Contact conditions
    double mu = 0.6;
    //double foot_length = 0.08;
    double foot_length = 0.14;
    this->contact_pyramid.resize(9, 5);
    this->contact_pyramid << 0., 0., -1., 0., 0.,
            1., 0., -mu/sqrt(2.), 0., 0.,
            -1., 0., -mu/sqrt(2.), 0., 0.,
            0., 1., -mu/sqrt(2.), 0., 0.,
            0., -1., -mu/sqrt(2.), 0., 0.,
            0., 0., -foot_length*mu, 0., 1.,
            0., 0., -foot_length*mu, 0, -1.,
            0., 0., -foot_length*mu, 1., 0.,
            0., 0., -foot_length*mu, -1., 0.;

    // Safe torque bounds
    this->torque_bounds.resize(10);
    this->torque_bounds << 112.5, 112.5, 195.2, 195.2, 45,
                           112.5, 112.5, 195.2, 195.2, 45;

    // Bounds on full qp decision variables
    this->lb.resize(40);
    this->lb << -INFTY * VectorXd::Ones(22),      // ddq
            -this->torque_bounds,    // torque
            -600., -600., 0., -400., -400., // f Foot
            -INFTY, -INFTY,                 // springs
            0;                              // qp relaxation delta
    this->ub.resize(40);
    this->ub << INFTY * VectorXd::Ones(22),       // ddq
            this->torque_bounds,     // torque
            600., 600., 1200., 400., 400.,  // f Foot
            INFTY, INFTY,                   // springs
            10000;                          // qp relaxation delta

    // Get all QP tunable parameters
    this->paramChecker.checkAndUpdateYaml("qp/reg_base_ddq", this->reg_base_ddq);
    this->paramChecker.checkAndUpdateYaml("qp/reg_stance_ddq", this->reg_stance_ddq);
    this->paramChecker.checkAndUpdateYaml("qp/reg_swing_ddq", this->reg_swing_ddq);
    this->paramChecker.checkAndUpdate("qp/nQPIter",       this->nQPIter);
    this->paramChecker.checkAndUpdateYaml("qp/reg_stance_u", this->reg_stance_u);
    this->paramChecker.checkAndUpdateYaml("qp/reg_swing_u", this->reg_swing_u);
    this->paramChecker.checkAndUpdate("qp/reg_rigid",     this->reg_rigid);
    this->paramChecker.checkAndUpdate("qp/reg_fx",        this->reg_fx);
    this->paramChecker.checkAndUpdate("qp/reg_fy",        this->reg_fy);
    this->paramChecker.checkAndUpdate("qp/reg_fz",        this->reg_fz);
    this->paramChecker.checkAndUpdate("qp/reg_muy",       this->reg_muy);
    this->paramChecker.checkAndUpdate("qp/reg_muz",       this->reg_muz);
    this->paramChecker.checkAndUpdate("qp/reg_clf_delta", this->reg_clf_delta);
    this->paramChecker.checkAndUpdate("qp/w_outputs",     this->w_outputs);

    this->paramChecker.checkAndUpdate("qp/w_hol_fixed", this->w_hol_fixed);
    this->paramChecker.checkAndUpdate("qp/w_hol_fx",    this->w_hol_fx);
    this->paramChecker.checkAndUpdate("qp/w_hol_fy",    this->w_hol_fy);
    this->paramChecker.checkAndUpdate("qp/w_hol_fz",    this->w_hol_fz);
    this->paramChecker.checkAndUpdate("qp/w_hol_my",    this->w_hol_my);
    this->paramChecker.checkAndUpdate("qp/w_hol_mz",    this->w_hol_mz);
    this->paramChecker.checkAndUpdate("qp/w_Vdot",    this->w_Vdot);

    this->paramChecker.checkAndUpdate("qp/clf_use_task_pd",    this->clf_use_task_pd);
    this->paramChecker.checkAndUpdate("qp/clf_use_inequality", this->clf_use_inequality);
    this->paramChecker.checkAndUpdate("qp/clf_use_Vdot_cost",  this->clf_use_Vdot_cost);
}

Walking1DControl::Walking1DControl(ros::NodeHandle &nh, cassie_model::Cassie &robot) :
    nh(&nh), ff(robot), tarsusSolver(robot) {
    // Main setup
    this->robot = &robot;

    // Create the QP solver
    qpsolver = new SQProblem(40, 32, HST_POSDEF); // (num vars, num constr)

    // Set QP Options
    Options options;
    options.setToMPC();
    options.enableRegularisation = BT_FALSE;//BT_FALSE;
    //options.setToDefault();
    options.printLevel = PL_LOW;//PL_MEDIUM;
    qpsolver->setOptions( options );

    // Parameter checker
    this->config.paramChecker.init(nh.getNamespace() + "/stepping");

    // All the storage
    this->cache.init();
    this->memory.init();
    this->config.init();

    // Service for calling reconfigures
    this->reconfigureService = nh.advertiseService("reconfigure_qp_stepping", &Walking1DControl::reconfigure, this);
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
    this->tarsusSolver.reset();
}

bool Walking1DControl::reconfigure() {
    std::cout << "Polling rosparams under: " << this->config.paramChecker.node.getNamespace() << std::endl;

    this->config.paramChecker.checkAndUpdate("raibert/swing_angle_absolute", this->config.swing_angle_absolute);
    this->config.paramChecker.checkAndUpdate("raibert/KpX", this->config.raibert_KpX);
    this->config.paramChecker.checkAndUpdate("raibert/KpY", this->config.raibert_KpY);
    this->config.paramChecker.checkAndUpdate("raibert/KdX", this->config.raibert_KdX);
    this->config.paramChecker.checkAndUpdate("raibert/KdY", this->config.raibert_KdY);
    this->config.paramChecker.checkAndUpdate("raibert/KiX", this->config.raibert_KiX);
    this->config.paramChecker.checkAndUpdate("raibert/KiY", this->config.raibert_KiY);

    this->config.paramChecker.checkAndUpdate("acceleration_regulation/ddq_KpX", this->config.ddq_KpX);
    this->config.paramChecker.checkAndUpdate("acceleration_regulation/ddq_KdX", this->config.ddq_KdX);
    this->config.paramChecker.checkAndUpdate("acceleration_regulation/ddq_KiX", this->config.ddq_KiX);
    this->config.paramChecker.checkAndUpdate("acceleration_regulation/ddq_KpY", this->config.ddq_KpY);
    this->config.paramChecker.checkAndUpdate("acceleration_regulation/ddq_KdY", this->config.ddq_KdY);
    this->config.paramChecker.checkAndUpdate("acceleration_regulation/ddq_KiY", this->config.ddq_KiY);

    this->config.paramChecker.checkAndUpdate("velocity_integrator_bleed_constant", this->config.velocity_integrator_bleed_constant);

    this->config.paramChecker.checkAndUpdate("vdx_ub", this->config.vdx_ub);
    this->config.paramChecker.checkAndUpdate("vdx_lb", this->config.vdx_lb);
    this->config.paramChecker.checkAndUpdate("vdy_ub", this->config.vdy_ub);
    this->config.paramChecker.checkAndUpdate("vdy_lb", this->config.vdy_lb);

    this->config.paramChecker.checkAndUpdate("force_regulation/Kp_grf_leglength", this->config.Kp_grf_leglength);
    this->config.paramChecker.checkAndUpdate("force_regulation/Kd_grf_leglength", this->config.Kd_grf_leglength);
    this->config.paramChecker.checkAndUpdate("force_regulation/k1_grf", this->config.k1_grf);
    this->config.paramChecker.checkAndUpdate("force_regulation/k2_grf", this->config.k2_grf);
    this->config.paramChecker.checkAndUpdate("force_regulation/k3_grf", this->config.k3_grf);
    this->config.paramChecker.checkAndUpdate("force_regulation/k4_grf", this->config.k4_grf);
    this->config.paramChecker.checkAndUpdate("force_regulation/k_grf_offset", this->config.k_grf_offset);

    this->config.paramChecker.checkAndUpdate("force_z_scale", this->config.force_z_scale);
    this->config.paramChecker.checkAndUpdate("force_sp_scale", this->config.force_sp_scale);

    this->config.reconfigure();
    this->phase.reconfigure(this->config.gaitlib.phaseRange, this->config.time_scale);

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

    this->phase.update(this->memory.timer.elapsed()*this->config.time_scale);

    if (!this->memory.isInitialized) {
        // Motion transition on a first loop
        this->memory.timer.restart();
        this->phase.update(this->memory.timer.elapsed()*this->config.time_scale);

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
        ROS_INFO("Initialized first step parameters");
    }

    // Rigidify swing leg measurements
    double lC=0.;
    double rC=0.;
    this->tarsusSolver.update();
    if (this->memory.iDomain == 0) {rC = 1.; lC = 0.;}
    if (this->memory.iDomain != 0) {rC = 0.; lC = 1.;}
    robot->q(LeftShinPitch)    *= lC;
    robot->q(LeftHeelSpring)   *= lC;
    robot->q(RightHeelSpring)  *= rC;
    robot->q(RightShinPitch)   *= rC;
    robot->dq(LeftShinPitch)   *= lC;
    robot->dq(LeftHeelSpring)  *= lC;
    robot->dq(RightHeelSpring) *= rC;
    robot->dq(RightShinPitch)  *= rC;
    robot->q(LeftTarsusPitch)   = lC*robot->q(LeftTarsusPitch) + (1.0 - lC)*this->tarsusSolver.getLeftRigidTarsusPosition();
    robot->q(RightTarsusPitch)  = rC*robot->q(RightTarsusPitch) + (1.0 - rC)*this->tarsusSolver.getRightRigidTarsusPosition();
    robot->dq(LeftTarsusPitch)  = lC*robot->dq(LeftTarsusPitch) + (1.0 - lC)*this->tarsusSolver.getLeftRigidTarsusVelocity();
    robot->dq(RightTarsusPitch) = rC*robot->dq(RightTarsusPitch) + (1.0 - rC)*this->tarsusSolver.getRightRigidTarsusVelocity();

    // Pull the radio outputs for movement
    bool requestTransition = (radio(SB) < 1.0);
    double xVd = this->config.vdx_ub * radio(LV);
    if (radio(LV) < 0.0)
        xVd = -this->config.vdx_lb * radio(LV);
    double yVd = -this->config.vdy_ub * radio(LH); // Left joystick = -1.0 but +y axis on robot
    double yawRate = -0.12 * radio(RH);            // Directionality swap for yaw as well
    this->memory.leg_angle_offset = 0.037 + 0.05 * radio(S1) + this->config.leg_angle_offset_center; // Swing leg angle offset
    this->memory.leg_roll_offset = 0.05 * radio(S2); // Swing leg roll offset

    // Do some engineering to make the robot smoother around pushes, with decreasing commanded velocity
    if ( (fabs(xVd) < 0.1) && (fabs(yVd) < 0.1) && fabs(this->lpVaXlastStep.getValue()) > 0.1 )
        xVd = this->lpVaXlastStep.getValue() * 0.75;
    if ( (fabs(xVd) < 0.1) && (fabs(yVd) < 0.1) && fabs(this->lpVaYlastStep.getValue()) > 0.1 )
        yVd = this->lpVaYlastStep.getValue() * 0.75;

    // Smooth the joystick commands
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
    double integral_max_x = 0.25;
    double integral_max_y = 0.1;
    double integral_x = this->memory.vd_integral_error(0) + ix * this->config.velocity_integrator_bleed_constant;
    double integral_y = this->memory.vd_integral_error(1) + iy * this->config.velocity_integrator_bleed_constant;
    if (integral_x > integral_max_x)
        this->memory.vd_integral_error(0) =  integral_max_x  - ix * this->config.velocity_integrator_bleed_constant;
    else if (integral_x < -integral_max_x)
        this->memory.vd_integral_error(0) = -integral_max_x  - ix * this->config.velocity_integrator_bleed_constant;
    else
        this->memory.vd_integral_error(0) = integral_x;

    if (integral_y > integral_max_y)
        this->memory.vd_integral_error(1) =  integral_max_y  - iy * this->config.velocity_integrator_bleed_constant;
    else if (integral_y < -integral_max_y)
        this->memory.vd_integral_error(1) = -integral_max_y  - iy * this->config.velocity_integrator_bleed_constant;
    else
        this->memory.vd_integral_error(1) = integral_y;

    this->memory.vd_integral_error(1) *= this->config.velocity_integrator_bleed_constant;
    this->memory.vd_integral_error(0) += ix;
    this->memory.vd_integral_error(1) += iy;

    // Get the outputs and update kinematics
    this->robot->kinematics.update(this->robot->model, this->robot->q, this->robot->dq);
    this->computeActual(this->cache.ya, this->cache.dya, this->cache.Dya, this->cache.DLfya);
    this->computeDesired(this->cache.yd, this->cache.dyd, this->cache.d2yd);

    // Augment polynomials for foot placement and dynamics feedback
    this->updateTurning(yawRate);
    this->updateRaibert(xVd, yVd);
    this->updateForces(xVd, yVd);
    this->updateAccelerations(xVd, yVd);

    // Compute controller
    if ( this->memory.isInitialized && this->config.use_qp )
        this->cache.u = this->getTorqueQP();
    else
        this->cache.u = this->getTorqueID();

    int ind = 0;
    for (int i=0; i<robot->iRotorMap.size(); ++i) {
        ind = robot->iRotorMap(i);
        this->cache.qmd(i) = this->robot->q(ind);
    }

    // Update guard
    this->computeGuard(requestTransition);

    // Return
    this->memory.lastTau = this->phase.tau;
    u = this->cache.u;

    // Update velocity average for step
    this->memory.velocityAllocator(0) += this->lpVaX.getValue();
    this->memory.velocityAllocator(1) += this->lpVaY.getValue();
    this->memory.nVelocitySamplesThisStep += 1;
}

VectorXd Walking1DControl::getTorqueQP() {
    // NOT YET RELEASED
    this->cache.u.setZero();
    // Return   
    return this->cache.u;
}

VectorXd Walking1DControl::getTorqueID() {
    // https://journals.sagepub.com/doi/pdf/10.1177/0278364910388677
    // Learning, planning, and control for quadruped locomotion over challenging terrain
    // Controller based on ^^
    MatrixXd J(9,10);
    if (this->memory.iDomain == 0) {
        SymFunction::J_yaRightStance(J, this->robot->q);
    } else {
        SymFunction::J_yaLeftStance(J, this->robot->q);
    }

    // Get output PD tracking torques
    this->cache.u = - J.completeOrthogonalDecomposition().solve(this->config.Kp.cwiseProduct(this->cache.ya - this->cache.yd) + this->config.Kd.cwiseProduct(this->cache.dya - this->cache.dyd));

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
    // QR decomposition
    HouseholderQR<MatrixXd> qr(Jc.transpose());
    MatrixXd Q, RO, R; Q.setZero(); R.setZero();
    RO = qr.matrixQR().triangularView<Upper>();
    Q = qr.householderQ();
    R = RO.block(0,0,11,11);

    // Feedforward
    VectorXd H(22);
    H = this->robot->dynamics.H*(this->cache.ddqd) + this->robot->dynamics.C;
    this->cache.uff = (this->config.Su*Q.transpose()*this->config.Be).completeOrthogonalDecomposition().solve(this->config.Su) * Q.transpose() * H;
    this->cache.u += this->cache.uff;

    // Compute GRF and force control
    VectorXd forces(11);
    forces = R.inverse() * this->config.Sc * Q.transpose() * (H - this->config.Be*this->cache.u);
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
    double moment_scale = 1.0;
    double foot_length = 0.14;
    double pitch_constr = moment_scale * forces(4) * foot_length / 2;
    if (this->memory.iDomain == 0) {
        this->cache.u(9) += this->config.u_stancetoe_offset*50.;
        if (this->cache.u(9) > pitch_constr)
            this->cache.u(9) = pitch_constr;
        if (this->cache.u(9) < -pitch_constr)
            this->cache.u(9) = -pitch_constr;
    } else {
        this->cache.u(4) += this->config.u_stancetoe_offset*50.;
        if (this->cache.u(4) > pitch_constr)
            this->cache.u(4) = pitch_constr;
        if (this->cache.u(4) < -pitch_constr)
            this->cache.u(4) = -pitch_constr;
    }

    // Return
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
    vax = this->lpVaX.getValue();
    vay = this->lpVaY.getValue();
    vprevx = this->lpVaXlastStep.getValue();
    vprevy = this->lpVaYlastStep.getValue();

    // Make swing leg respond to pitch/roll error
    double offset = 0;
    if (this->memory.iDomain == 0)
        offset = -0.02;
    else
        offset = 0.02;

    // // Leg pitch
    double lp_float = 0.;
    double lr_float = 0.;
    double dlp_float = 0.;
    double dlr_float = 0.;
    if (this->config.swing_angle_absolute) {
        // Make swing leg respond to pitch/roll error

        // Leg pitch
        lp_float  = (this->cache.ya(1) - this->cache.yd(1));
        dlp_float = (this->cache.dya(1) - this->cache.dyd(1));
        // Hip roll
        lr_float  = - (this->cache.ya(0) - this->cache.yd(0));
        dlr_float = - (this->cache.dya(0) - this->cache.dyd(0));

        // Leg pitch
        this->cache.yd(5)  += (this->cache.ya(1) - this->cache.yd(1));
        this->cache.dyd(5) += (this->cache.dya(1) - this->cache.dyd(1));
        // Hip roll
        this->cache.yd(6)  -= (this->cache.ya(0) - this->cache.yd(0)) + offset;
        this->cache.dyd(6) -= (this->cache.dya(0) - this->cache.dyd(0));
    }

    // Raibert
    double xo, yo;
    xo = this->config.raibert_KpX * (vax - xVd)
         + this->config.raibert_KdX * (vax - vprevx)
         + this->config.raibert_KiX * this->memory.vd_integral_error(0);
    xo = control_utilities::clamp(xo, -0.6, 0.6);
    xo = this->rateRaibertX.update(0.001, xo);

    yo = this->config.raibert_KpY * (vay - yVd) // scaled as it overshoots
         + this->config.raibert_KdY * (vay - vprevy)
         + this->config.raibert_KiX * this->memory.vd_integral_error(1);
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
            0.,lp_float+(lp - lp_),lp_float+(lp - lp_),lp_float+(lp - lp_),lp_float+(lp - lp_),
            0.,lr_float+(lr - lr_),lr_float+(lr - lr_),lr_float+(lr - lr_),lr_float+(lr - lr_);

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

   // Update the accelerations
   this->cache.ddqd(BaseRotX) -= - d2_newOutputs(2);
   this->cache.ddqd(BaseRotY) -=   d2_newOutputs(1);
   if (this->memory.iDomain == 0) {
       this->cache.ddqd(LeftKneePitch) += d2_newOutputs(0) * (1./this->cache.Dya(4,LeftKneePitch));
       this->cache.ddqd(LeftKneePitch) += d2_newOutputs(1) * (1./this->cache.Dya(5,LeftKneePitch));
       this->cache.ddqd(LeftHipPitch)  += d2_newOutputs(1) * (1./this->cache.Dya(5,LeftHipPitch));
       this->cache.ddqd(LeftHipRoll)   += d2_newOutputs(2) * (1./this->cache.Dya(6,LeftHipRoll));
   } else {
       this->cache.ddqd(RightKneePitch) += d2_newOutputs(0) * (1./this->cache.Dya(4,RightKneePitch));
       this->cache.ddqd(RightKneePitch) += d2_newOutputs(1) * (1./this->cache.Dya(5,RightKneePitch));
       this->cache.ddqd(RightHipPitch)  += d2_newOutputs(1) * (1./this->cache.Dya(5,RightHipPitch));
       this->cache.ddqd(RightHipRoll)   += d2_newOutputs(2) * (1./this->cache.Dya(6,RightHipRoll));
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

void Walking1DControl::updateForces(double xVd, double yVd) {
    // Compute ddq PD control
    VectorXd legPitch(1), legPitchVelocity(1), legRoll(1), legRollVelocity(1), poseStance(6);
    if (this->memory.iDomain == 0) {
        VectorWrap legPitch_(legPitch), legPitchVelocity_(legPitchVelocity), legRoll_(legRoll), legRollVelocity_(legRollVelocity), poseStance_(poseStance);
        SymFunction::rightLegPitch(legPitch_, this->robot->q);
        SymFunction::rightLegPitchVelocity(legPitchVelocity_, this->robot->q, this->robot->dq);
        SymFunction::rightLegRoll(legRoll_, this->robot->q);
        SymFunction::rightLegRollVelocity(legRollVelocity_, this->robot->q, this->robot->dq);
        SymFunction::pose_rightFoot(poseStance_, this->robot->q);
    } else {
        VectorWrap legPitch_(legPitch), legPitchVelocity_(legPitchVelocity), legRoll_(legRoll), legRollVelocity_(legRollVelocity), poseStance_(poseStance);
        SymFunction::leftLegPitch(legPitch_, this->robot->q);
        SymFunction::leftLegPitchVelocity(legPitchVelocity_, this->robot->q, this->robot->dq);
        SymFunction::leftLegRoll(legRoll_, this->robot->q);
        SymFunction::leftLegRollVelocity(legRollVelocity_, this->robot->q, this->robot->dq);
        SymFunction::pose_leftFoot(poseStance_, this->robot->q);
    }
    legPitch(0) -= 0.035; // neutral constant offset found by trial & error

    double pax, pay, paz;
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
    paz = - pose_sf(2);

    double x = - this->cache.ya(3)*sin(legPitch(0));
    double x0 = fabs(xVd * 0.4 * 0.5);
    double xp = x / x0;
    if (fabs(xVd) < 0.02)
        xp = 0;

    double y = this->cache.ya(3)*sin(legPitch(0)) * sin(legRoll(0));
    double y0 = fabs(yVd * 0.4 * 0.5);
    double yp = y / y0;
    if (fabs(yVd) < 0.02)
        yp = 0;

    // Virtual leg length regulation
    double offset = this->config.k_grf_offset;
    double k1 = this->config.k1_grf;
    double k2 = this->config.k2_grf;
    double k3 = this->config.k3_grf;
    double k4 = this->config.k4_grf;
    double length_desired = this->cache.yd(3) + offset;

    double delta_length_x = 0;
    if (xp > 0)
        delta_length_x += xp*(k1*xVd + k2*(xVd - this->lpVaX.getValue()));
    else
        delta_length_x += xp*(k3*xVd + k4*(xVd - this->lpVaX.getValue()));

    // Maximum leg delta is dependent on current speed
    double max_delta_x = 0.04 * fabs(this->lpVaX.getValue());
    if (delta_length_x < -max_delta_x)
        delta_length_x = -max_delta_x;
    if (delta_length_x >  max_delta_x)
        delta_length_x =  max_delta_x;
    length_desired += delta_length_x;

    double e_length = this->cache.ya(3) - length_desired;
    double de_length = this->cache.dya(3) - this->cache.dyd(3);

    this->cache.Fd(2) -= (this->config.Kp_grf_leglength * e_length * sin(legPitch(0))
                         + this->config.Kd_grf_leglength * (de_length * sin(legPitch(0))
                                                  + e_length * cos(legPitch(0))*legPitchVelocity(0) ));
    this->cache.Fd(3) -= ( - this->config.Kp_grf_leglength * e_length * sin(legRoll(0)) * cos(legPitch(0))
                         + this->config.Kd_grf_leglength * (- de_length * sin(legRoll(0)) * cos(legPitch(0))
                                                  - e_length * cos(legRoll(0)) * cos(legPitch(0))*legRollVelocity(0)
                                                  + e_length * sin(legRoll(0)) * sin(legPitch(0))*legPitchVelocity(0)));
    this->cache.Fd(4) -= (this->config.Kp_grf_leglength * e_length * cos(legRoll(0)) * cos(legPitch(0))
                         + this->config.Kd_grf_leglength * (de_length * cos(legRoll(0)) * cos(legPitch(0))
                                                  - e_length * sin(legRoll(0)) * cos(legPitch(0)) * legRollVelocity(0)
                                                  - e_length * cos(legRoll(0)) * sin(legPitch(0)) * legPitchVelocity(0) ));
}

void Walking1DControl::nextDomain() {
    // Increment the domain indexing
    this->memory.iDomain += 1;
    if ( this->memory.iDomain >= this->config.nDomain )
        this->memory.iDomain = 0;
    this->memory.timer.restart();

    // Update Raibert controller things
    this->memory.lastStepVelocity = this->memory.velocityAllocator / (double)this->memory.nVelocitySamplesThisStep;
    this->memory.velocityAllocator.setZero();
    this->memory.nVelocitySamplesThisStep = 0;

    // Store terminal raibert offset
    this->memory.raibert_offset_last << this->cache.raibert_offset;
    this->memory.u_prev.setZero();
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
    double s = this->phase.tau;
    Vector2d phaseRange = this->phase.getPhaseRange();
    VectorXd col1(9), col2(9);
    col1 << this->memory.yd_last;
    col2 << this->memory.yd_last + this->memory.dyd_last * ( (phaseRange(1) - phaseRange(0)) / (this->memory.paramCurrent.cols() - 1)  );
    
    // Stance yaw
    this->memory.paramCurrent(2,0) = col1(2);
    this->memory.paramCurrent(2,1) = col2(2);

    // Swing angle
    this->memory.paramCurrent(5,0) = col1(5);
    this->memory.paramCurrent(5,1) = col2(5);

    // Swing roll
    this->memory.paramCurrent(6,0) = col1(6);
    this->memory.paramCurrent(6,1) = col2(6);

    // Swing yaw
    this->memory.paramCurrent(7,0) = col1(7);
    this->memory.paramCurrent(7,1) = col2(7);

    // Compute
    bezier_tools::bezier(this->memory.paramCurrent, s, yd);
    bezier_tools::dbezier(this->memory.paramCurrent, s, dyd);
    bezier_tools::dbezier(this->memory.paramCurrent, s, d2yd);
    dyd = dyd * this->phase.dtau;
    d2yd = d2yd * pow(this->phase.dtau,2);

    // Use the impact leg extension for the following step
    yd(3) += (1.0 - s) * this->memory.raibert_offset_last(0);

    // Add leg roll offset
    yd(6) += this->memory.leg_roll_offset;

    // Add swing foot offset
    yd(8) += this->config.swing_foot_offset;

    // Add leg angle offset
    yd(5) += this->memory.leg_angle_offset;
}

void Walking1DControl::computeGuard(bool requestTransition) {
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

        // Increment
        this->nextDomain();

        // Update terminal outputs
        // Compute here manually rather than using methods to avoid extra overhead
        if (this->memory.iDomain == 0) {
            VectorWrap ya_(this->cache.ya), dya_(this->cache.dya);
            SymFunction::yaRightStance(ya_, this->robot->q);
            SymFunction::dyaRightStance(dya_, this->robot->q, this->robot->dq);
        } else {
            VectorWrap ya_(this->cache.ya), dya_(this->cache.dya);
            SymFunction::yaLeftStance(ya_, this->robot->q);
            SymFunction::dyaLeftStance(dya_, this->robot->q, this->robot->dq);
        }
        VectorXd yd(9), dyd(9);
        bezier_tools::bezier(this->memory.paramCurrent, this->phase.tau, yd);
        bezier_tools::dbezier(this->memory.paramCurrent, this->phase.tau, dyd);
        this->cache.dyd *= this->phase.dtau;

        this->memory.yd_last <<
                yd(0), // base roll
                yd(1), // base pitch
                this->cache.ya(2), // stance yaw
                yd(3), // stance leg length
                yd(4), // swing leg length
                this->cache.ya(5) - this->memory.leg_angle_offset,    // Leg angle
                this->cache.ya(6) - this->memory.leg_roll_offset, // leg roll
                this->cache.ya(7), // swing yaw
                this->cache.ya(8); // swing foot
        this->memory.dyd_last <<
                dyd(0),
                dyd(1),
                this->cache.dya(2),
                dyd(3),
                dyd(4),
                this->cache.dya(5),
                this->cache.dya(6),
                this->cache.dya(7),
                this->cache.dya(8);

        // If the new domain is right stance and stop was requested, then queue in the right to stance parameters
        if ((this->memory.iDomain == 0) && (requestTransition) && (stoppable)) {
            this->memory.queueStop = true;
            //this->memory.paramCurrent << this->config.paramRightToStand;
        }
    }
}

void Walking1DControl::computeLibrary() {
    // Get the desired
    double v_step_x = control_utilities::clamp(this->lpVaXlastStep.getValue(), -0.8, 1.9);
    double v_step_y = control_utilities::clamp(this->lpVaYlastStep.getValue(), -0.5, 0.5);

    // Interpolate the gait
    VectorXd tempa(this->config.gaitlib.right.amat_array[0][0].cols());
    VectorXd tempddq(this->config.gaitlib.right.avelocity_array[0][0].cols());
    VectorXd temppd(this->config.gaitlib.right.addq_array[0][0].cols());
    VectorXd tempvd(this->config.gaitlib.right.addq_array[0][0].cols());
    VectorXd tempFd(this->config.gaitlib.right.af_array[0][0].cols());
    VectorXd tempud(this->config.gaitlib.right.au_array[0][0].cols());
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
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.right.au_array,
                                               v_step_x, v_step_y, tempud);
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
        cassie_common_toolbox::bilinear_interp(this->config.gaitlib.vd_x, this->config.gaitlib.vd_y, this->config.gaitlib.left.au_array,
                                               v_step_x, v_step_y, tempud);
    }

    // Reshape parameters
    Map<MatrixXd> tempM(tempa.data(),     9,7);
    Map<MatrixXd> tempDDQ(tempddq.data(), 22,7);
    Map<MatrixXd> tempP(temppd.data(),    2,7);
    Map<MatrixXd> tempV(tempvd.data(),    2,7);
    Map<MatrixXd> tempF(tempFd.data(),    7,7);
    Map<MatrixXd> tempU(tempud.data(),    10,7);

    this->memory.paramCurrent << tempM;
    bezier_tools::bezier(tempP,                           this->phase.tau, this->cache.pd);
    bezier_tools::bezier(tempV*this->config.time_scale,   this->phase.tau, this->cache.vd);
    bezier_tools::bezier(tempDDQ*this->config.time_scale, this->phase.tau, this->cache.ddqd);
    bezier_tools::bezier(tempF,                           this->phase.tau, this->cache.Fd);
    bezier_tools::bezier(tempU,                           this->phase.tau, this->cache.uff);

    // Scale Z force
    this->cache.Fd(4) *= this->config.force_z_scale;
}

void Walking1DControl::getDebug(VectorXf &dbg) {
    double tsec = static_cast<double>(ros::Time::now().sec);
    double tnsec = 1e-9 * static_cast<double>(ros::Time::now().nsec);
    // Move zero to closer time rather than 1970 so it fits in a float
    // Still bad practice to put time in floats, but this is just for logging
    if ( !this->config.isSim )
        tsec -= 1.6074545e9;

    // Use floats for logging size and speed
    dbg << static_cast<float>(tsec), static_cast<float>(tnsec), // 2
           static_cast<float>(this->phase.tau),   // 1
           static_cast<float>(this->phase.dtau),  // 1
           this->cache.ya.cast <float> (),        // 9
           this->cache.dya.cast <float> (),       // 9
           this->cache.yd.cast <float> (),        // 9
           this->cache.dyd.cast <float> (),       // 9
           static_cast<float>(this->cache.V),     // 1
           this->cache.u.cast <float> (),         // 10
           this->cache.Fdes.cast <float> (),      // 11
           static_cast<float>(this->cache.delta), // 1
           static_cast<float>(this->cache.vd[0]), static_cast<float>(this->cache.vd[1]),   // 2
           static_cast<float>(this->robot->dq[0]), static_cast<float>(this->robot->dq[1]), // 2
           static_cast<float>(this->lpVaXlastStep.getValue()), static_cast<float>(this->lpVaYlastStep.getValue()), //2
           this->cache.raibert_offset.cast <float> (), // 3
           this->cache.uff.cast <float> ();            // 10
}

int Walking1DControl::SwingLegInverseKinematics(VectorXd &qmd, VectorXd &dqmd, VectorXd &ddqmd) {
    int iteration_limit = 100;
    double xtol = 1e-16;
    VectorXd q(22), dq(22);
    q << robot->q;
    dq << robot->dq;
    VectorXi iActive(5);
    VectorXi uiActive(5);
    if (this->memory.iDomain == 0) {
        iActive << LeftHipRoll, LeftHipYaw, LeftHipPitch, LeftKneePitch, LeftFootPitch;
        uiActive << 0,1,2,3,4;
    } else {
        iActive << RightHipRoll, RightHipYaw, RightHipPitch, RightKneePitch, RightFootPitch;
        uiActive <<  5,6,7,8,9;
    }

    VectorXd F(5);
    MatrixXd J(5,5);
    double f_d = 0.0;

    MatrixXd Ffull(9,1), Jfull(9,10);
    for (int i=0; i<iteration_limit; ++i){
        // Reinitialize the state at the current guess
        if (this->memory.iDomain == 0) {
            for (int j=0; j<5; ++j)
                q(iActive(j)) = qmd(j);

            SymFunction::yaRightStance(Ffull, q);
            SymFunction::J_yaRightStance(Jfull, q);

            F << Ffull.block(4,0, 5,1);
            J << Jfull.block(4,0, 5,5);
        } else {
            for (int j=0; j<5; ++j)
                q(iActive(j)) = qmd(j);

            SymFunction::yaLeftStance(Ffull, q);
            SymFunction::J_yaLeftStance(Jfull, q);

            F << Ffull.block(4,0, 5,1);
            J << Jfull.block(4,5, 5,5);
        }

        F -= this->cache.yd.segment(4,5);

        // Check for completion
        f_d = 0.5 * F.transpose() * F;
        if (f_d < xtol) {
            // Get velocity
            dqmd << J.completeOrthogonalDecomposition().solve(this->cache.dyd.segment(4,5));

            // Get acceleration
            for (int j=0; j<5; ++j)
                dq(iActive(j)) = dqmd(j);

            MatrixXd dJfull(9,10), dJ(5,5);
            if (this->memory.iDomain == 0) {
                SymFunction::Jdot_yaRightStance(dJfull, q, dq);
                 dJ << dJfull.block(4,0, 5,5);
            } else {
                SymFunction::Jdot_yaLeftStance(dJfull, q, dq);
                dJ << dJfull.block(4,5, 5,5);
            }
            ddqmd << J.completeOrthogonalDecomposition().solve( this->cache.d2yd.segment(4,5) - dJ*dqmd );
            return i;
        }

        // Perform the update
        qmd = qmd - (J.transpose() * (J * J.transpose()).inverse()) * F;
    }
    ROS_WARN("IK DID NOT CONVERGE");
    return -1;
}
