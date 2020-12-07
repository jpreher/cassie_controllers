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

#include <cassie_controllers/standing_control_qp.hpp>
#include <cassie_common_toolbox/RadioButtonMap.hpp>
#include <ros/ros.h>
#include <control_utilities/limits.hpp>

using namespace Eigen;
using namespace cassie_model;
using namespace control_utilities;
USING_NAMESPACE_QPOASES

void StandingControlQP::Cache::init() {
    // Initialize outputs
    this->ya.resize(6);
    this->dya.resize(6);
    this->Dya.resize(6,44);
    this->DLfya.resize(6,44);
    this->yd.resize(6);
    this->dyd.resize(6);
    this->d2yd.resize(6);
    this->u.resize(10);
    this->qmd.resize(10);
    this->dqmd.resize(10);
    this->eta.resize(12);

    // Size all QP matrices for speed
    this->Jc.resize(16,22);
    this->dJc.resize(16,22);
    this->A_holonomics.resize(16,49);
    this->b_holonomics.resize(16);
    this->Aeq_dynamics.resize(22, 49);
    this->beq_dynamics.resize(22);
    this->A_chatter.resize(10,49);
    this->b_chatter.resize(10);
    this->A_friction.resize(18,49);
    this->b_lb_friction.resize(18);
    this->b_ub_friction.resize(18);
    this->A_y.resize(6,49);
    this->b_y.resize(6);
    this->A_clf.resize(1,49);
    this->A_reg.resize(49,49);
    this->b_reg.resize(49);
    this->qpsol.resize(49);
    this->Fdes.resize(16);

    // Inverse kinematics
    this->dq_desired.resize(10);
    this->target_velocities.resize(10);
    this->IK_solution.resize(10);
    this->Jik.resize(10,10);
    this->pLF_actual.resize(5,1);
    this->pRF_actual.resize(5,1);

    // Reset
    this->reset();
}

void StandingControlQP::Cache::reset() {
    // Outputs
    this->ya.setZero();
    this->dya.setZero();
    this->Dya.setZero();
    this->DLfya.setZero();
    this->yd.setZero();
    this->dyd.setZero();
    this->d2yd.setZero();
    this->dqmd.setZero();
    this->qmd.setZero();
    this->T_des_pelvis.setIdentity();
    this->euler = EulerAnglesZYXd(0.0, 0.0, 0.0);
    this->u.setZero();
    this->eta.setZero();

    // QP terms
    this->Jc.setZero();
    this->dJc.setZero();
    this->A_holonomics.setZero();
    this->b_holonomics.setZero();
    this->Aeq_dynamics.setZero();
    this->beq_dynamics.setZero();
    this->A_chatter.setZero();
    this->b_chatter.setZero();
    this->A_friction.setZero();
    this->b_lb_friction.setZero();
    this->b_ub_friction.setZero();
    this->A_y.setZero();
    this->b_y.setZero();
    this->A_clf.setZero();
    this->A_reg.setZero();
    this->A_reg << MatrixXd::Identity(49,49);
    this->b_reg.setZero();
    this->qpsol.setZero();
    this->V = 0.;
    this->Fdes.setZero();
    this->delta = 0.;

    // Inverse kinematics terms
    this->dq_desired.setZero();
    this->IK_solution.setZero();
    this->T_des_pelvis.setIdentity();
    this->T_des_leftFoot.setIdentity();
    this->T_des_rightFoot.setIdentity();
    this->euler = EulerAnglesZYXd(0.0, 0.0, 0.0);
    this->target_velocities.setZero();
    this->leftTwist.setZero();
    this->rightTwist.setZero();
    this->pitch_desired = 0;
    this->pLF_actual.setZero();
    this->pRF_actual.setZero();
}

void StandingControlQP::Memory::init() {
    this->u_prev.resize(10);
    this->IK_solution_prev.resize(10);
    this->dyd_last.resize(6);
    this->reset();
}

void StandingControlQP::Memory::reset() {
    this->mode = -1;
    this->contact_initialized = false;
    this->readyToTransition = false;
    this->queueTransition = false;
    this->T_des_prev_leftFoot.setIdentity();
    this->T_des_prev_rightFoot.setIdentity();
    this->T_leftContact.setIdentity();
    this->T_rightContact.setIdentity();
    this->u_prev.setZero();
    this->IK_solution_prev.setZero();
    this->spoolup = 0.;
    this->dyd_last.setZero();
    this->qp_initialized = false;
    this->timer.restart();

    this->crouchOverrideCompleted = false;
    this->crouchOverrideInitialized = false;
    this->crouchOverrideTimer.restart();
}

void StandingControlQP::Config::init() {
    this->control_rate = 0.0005;
    this->height_lowpass_dt_cutoff = 100.0;
    this->lateral_lowpass_dt_cutoff = 100.0;
    this->pitch_lowpass_dt_cutoff = 100.0;
    this->x_com_offset = 0;
    this->pitch_lb = -0.1;
    this->pitch_ub = 0.1;
    this->height_lb = -0.7;
    this->height_ub = -0.4;
    this->lateral_lb = -0.2;
    this->lateral_ub = 0.2;

    this->Kp.resize(10);
    this->Kd.resize(10);
    this->Kpy.resize(6);
    this->Kdy.resize(6);
    this->Pdiag.resize(12);
    this->Poffdiag.resize(6);
}

void StandingControlQP::Config::reconfigure() {
    this->paramChecker.checkAndUpdate("/cassie/locomotion_control/dt", this->control_rate);
    this->paramChecker.checkAndUpdate("use_qp", this->use_qp);

    this->paramChecker.checkAndUpdate("inverse_kinematics/ik_xtol", this->ik_xtol);
    this->paramChecker.checkAndUpdate("inverse_kinematics/ik_iter_limit", this->ik_iter_limit);

    this->paramChecker.checkAndUpdate("pose_command/x_com_offset", this->x_com_offset);
    this->paramChecker.checkAndUpdate("pose_command/height_lowpass_dt_cutoff", this->height_lowpass_dt_cutoff);
    this->paramChecker.checkAndUpdate("pose_command/lateral_lowpass_dt_cutoff", this->lateral_lowpass_dt_cutoff);
    this->paramChecker.checkAndUpdate("pose_command/pitch_lowpass_dt_cutoff", this->pitch_lowpass_dt_cutoff);
    this->paramChecker.checkAndUpdate("pose_command/x_lowpass_dt_cutoff", this->x_lowpass_dt_cutoff);
    this->paramChecker.checkAndUpdate("pose_command/height_lb", this->height_lb);
    this->paramChecker.checkAndUpdate("pose_command/height_ub", this->height_ub);
    this->paramChecker.checkAndUpdate("pose_command/lateral_lb", this->lateral_lb);
    this->paramChecker.checkAndUpdate("pose_command/lateral_ub", this->lateral_ub);
    this->paramChecker.checkAndUpdate("pose_command/pitch_lb", this->pitch_lb);
    this->paramChecker.checkAndUpdate("pose_command/pitch_ub", this->pitch_ub);

    this->paramChecker.checkAndUpdate("use_lateral_comp", this->use_lateral_comp);
    this->paramChecker.checkAndUpdate("Kp_lateral_compensator", this->Kp_lateral_compensator);
    this->paramChecker.checkAndUpdate("Kd_lateral_compensator", this->Kd_lateral_compensator);

    // Precomputed CLF terms and configurable gains
    this->paramChecker.checkAndUpdate("qp/res_clf_ep", this->res_clf_ep);
    this->paramChecker.checkAndUpdate("qp/clf_gam", this->clf_gam);
    this->ny  = 6;
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
    this->P.topRightCorner(6,6).diagonal() << this->Poffdiag;
    this->P.bottomLeftCorner(6,6).diagonal() << this->Poffdiag;

    MatrixXd Iep(2*ny,2*ny);
    Iep.setZero();
    Iep.topLeftCorner(ny,ny) << MatrixXd::Identity(ny,ny) * (1.0/this->ep);
    Iep.bottomRightCorner(ny,ny) << MatrixXd::Identity(ny,ny);
    this->P = Iep.transpose() * this->P * Iep;

    // Derivative of linear output dynamics
    this->LFV_mat = this->F.transpose() * this->P + this->P * this->F;

    // Robot torque mapping with gear reduction
    this->Be.resize(22,10);
    this->Be.setZero();
    this->Be(LeftHipRoll,    0) = 25.;
    this->Be(LeftHipYaw,     1) = 25.;
    this->Be(LeftHipPitch,   2) = 16.;
    this->Be(LeftKneePitch,  3) = 16.;
    this->Be(LeftFootPitch,  4) = 50.;
    this->Be(RightHipRoll,   5) = 25.;
    this->Be(RightHipYaw,    6) = 25.;
    this->Be(RightHipPitch,  7) = 16.;
    this->Be(RightKneePitch, 8) = 16.;
    this->Be(RightFootPitch, 9) = 50.;

    // Contact conditions
    double mu = 0.6;
    double foot_length = 0.14;
    this->contact_pyramid.resize(9, 5);
    this->contact_pyramid << 0., 0., -1., 0., 0.,
            1., 0., -2.*mu/sqrt(2.), 0., 0.,
            -1., 0., -mu/sqrt(2.), 0., 0.,
            0., 1., -mu/sqrt(2.), 0., 0.,
            0., -1., -mu/sqrt(2.), 0., 0.,
            0., 0., -foot_length/2., 0., 1.,
            0., 0., -foot_length/2., 0, -1.,
            0., 0., -foot_length/2., 1., 0.,
            0., 0., -foot_length/2., -1., 0.;

    // Safe torque bounds
    this->torque_bounds.resize(10);
    this->torque_bounds << 4.5, 4.5, 12.2, 12.2, 0.9,
            4.5, 4.5, 12.2, 12.2, 0.9;

    // Get all QP tunable parameters
    this->paramChecker.checkAndUpdate("qp/clf_use_task_pd",    this->clf_use_task_pd);
    this->paramChecker.checkAndUpdate("qp/clf_use_inequality", this->clf_use_inequality);
    this->paramChecker.checkAndUpdate("qp/clf_use_Vdot_cost",  this->clf_use_Vdot_cost);
    this->paramChecker.checkAndUpdate("qp/nQPIter",            this->nQPIter);
    this->paramChecker.checkAndUpdate("qp/reg_ddq",            this->reg_ddq);
    this->paramChecker.checkAndUpdate("qp/reg_u",              this->reg_u);
    this->paramChecker.checkAndUpdate("qp/reg_achilles",       this->reg_achilles);
    this->paramChecker.checkAndUpdate("qp/reg_rigid",          this->reg_rigid);
    this->paramChecker.checkAndUpdate("qp/reg_fx",             this->reg_fx);
    this->paramChecker.checkAndUpdate("qp/reg_fy",             this->reg_fy);
    this->paramChecker.checkAndUpdate("qp/reg_fz",             this->reg_fz);
    this->paramChecker.checkAndUpdate("qp/reg_muy",            this->reg_muy);
    this->paramChecker.checkAndUpdate("qp/reg_muz",            this->reg_muz);
    this->paramChecker.checkAndUpdate("qp/reg_clf_delta",      this->reg_clf_delta);
    this->paramChecker.checkAndUpdate("qp/w_u_chatter",        this->w_u_chatter);
    this->paramChecker.checkAndUpdate("qp/w_outputs",          this->w_outputs);
    this->paramChecker.checkAndUpdate("qp/w_Vdot",             this->w_Vdot);

    this->paramChecker.checkAndUpdate("qp/w_hol_achilles", this->w_hol_achilles);
    this->paramChecker.checkAndUpdate("qp/w_hol_fixed",    this->w_hol_fixed);
    this->paramChecker.checkAndUpdate("qp/w_hol_fx",       this->w_hol_fx);
    this->paramChecker.checkAndUpdate("qp/w_hol_fy",       this->w_hol_fy);
    this->paramChecker.checkAndUpdate("qp/w_hol_fz",       this->w_hol_fz);
    this->paramChecker.checkAndUpdate("qp/w_hol_my",       this->w_hol_my);
    this->paramChecker.checkAndUpdate("qp/w_hol_mz",       this->w_hol_mz);

    this->paramChecker.checkAndUpdateYaml("qp/kp", this->Kpy);
    this->paramChecker.checkAndUpdateYaml("qp/kd", this->Kdy);
}

bool StandingControlQP::reconfigure() {
    std::cout << "Polling rosparams under: " << this->config.paramChecker.node.getNamespace() << std::endl;

    // Reconfigure high-level controller params
    this->config.reconfigure();

    // Reconfigure the IK PD controller
    VectorXd Kp(10), Kd(10);
    VectorXd Kph(5), Kdh(5); Kph.setZero(); Kdh.setZero();
    this->config.paramChecker.checkAndUpdateYaml("kp", Kph);
    this->config.paramChecker.checkAndUpdateYaml("kd", Kdh);
    Kp << Kph, Kph;
    Kd << Kdh, Kdh;

    // Pass in the new values
    this->pd.reconfigure(Kp, Kd);
    this->heightFilter.reconfigure(config.control_rate, config.height_lowpass_dt_cutoff);
    this->lateralFilter.reconfigure(config.control_rate, config.lateral_lowpass_dt_cutoff);
    this->pitchFilter.reconfigure(config.control_rate, config.pitch_lowpass_dt_cutoff);
    this->leftXFilter.reconfigure(config.control_rate, config.x_lowpass_dt_cutoff);
    this->rightXFilter.reconfigure(config.control_rate, config.x_lowpass_dt_cutoff);

    return true;
}

bool StandingControlQP::reconfigure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    this->reconfigure();
    return true;
}

void StandingControlQP::reset() {
    this->cache.reset();
    this->memory.reset();

    this->heightFilter.reset();
    this->lateralFilter.reset();
    this->pitchFilter.reset();
}

StandingControlQP::StandingControlQP(ros::NodeHandle &nh, cassie_model::Cassie &robot) : nh(&nh)
{
    // Main setup
    this->robot = &robot;

    // Create the QP solver
    int nvar = 49;
    int ncon = 41;
    qpsolver = new SQProblem(nvar, ncon, HST_POSDEF); // (num vars, num constr), Hessian is pos def by formulation

    // Set QP Options
    Options options;
    options.setToMPC(); // Fastest stable preset for qpoases
    options.enableRegularisation = BT_FALSE; // We do our own hand-tuned regulatisation
    options.printLevel = PL_LOW; // Change for additional readouts: PL_MEDIUM, PL_HIGH
    qpsolver->setOptions( options );

    // Parameter checker
    this->config.paramChecker.init(nh.getNamespace() + "/standing");

    // All the storage
    this->cache.init();
    this->memory.init();
    this->config.init();

    // Service for calling reconfigures
    this->reconfigureService = nh.advertiseService("reconfigure_standing", &StandingControlQP::reconfigure, this);
    this->reconfigure();

    // Inverse kinematics
    this->IKfunction.init(robot);
}

void StandingControlQP::update(VectorXd &radio, VectorXd &u) {
    // Reset the transition flag
    this->memory.readyToTransition = false;

    // Set base position to zero in case proprioception hasnt updated since last loop
    this->robot->q.segment(BasePosX,3) << 0.,0.,0.;

    // Update spooling
    double dt = control_utilities::clamp(this->memory.timer.elapsed(), this->config.control_rate/4, this->config.control_rate*4);
    this->memory.timer.restart();
    this->memory.spoolup = this->memory.spoolup * (1 - 0.0020) + 1 * 0.0020;

    // Run the ID controller in the air until contact is initialized
    if ( (this->robot->leftContact >= 0.99) && (this->robot->rightContact >= 0.99) )
        this->memory.contact_initialized = true;

    // Current joint configurations. Store to IK sol in case it does not converge.
    VectorXd q_motors(10), dq_motors(10);
    for (int i = 0; i < this->robot->iRotorMap.size(); ++i) {
        int ind = this->robot->iRotorMap(i);
        q_motors(i) = this->robot->q(ind);
        dq_motors(i) = this->robot->dq(ind);
        this->cache.IK_solution(i) = q_motors(i);
    }

    // Get current foot positions (using deflected legs!)
    SymFunction::p_leftSole_constraint(this->cache.pLF_actual, this->robot->q);
    SymFunction::p_rightSole_constraint(this->cache.pRF_actual, this->robot->q);

    // If initializing, set foot configurations
    if (this->memory.mode == -1) {
        this->setFootTransforms();
        this->memory.mode = 0;
        this->memory.IK_solution_prev << this->cache.IK_solution;
    }

    // Process the radio transmissions
    this->processRadio(radio);

    // Compute the actual and desired outputs
    this->computeDesired(radio);
    this->cache.yd(0) -= 0.0068 * this->cache.yd(2) - 0.02562; // Linear fit of COM on range of motion...
    this->computeActual();

    // Compute controller
    if ( this->memory.contact_initialized && this->config.use_qp ) {
        u = this->getTorqueQP();
    } else {
        this->runInverseKinematics(this->cache.pLF_actual, this->cache.pRF_actual);
        u = this->getTorqueID();
    }

    // Check if we are at the end of the transition phase...
    // This is implemented for transitioning to our walking controller. Not currently released.
    if (this->memory.queueTransition) {
        if ( (this->heightFilter.getValue() < 0.95 * this->config.height_lb) ) {
            this->memory.readyToTransition = true;
        }
    }
}

void StandingControlQP::computeActual() {
    // Targets
    VectorXd qb(6);
    qb << this->robot->q.segment(BasePosX,6);
    qb.segment(BasePosX,3) = -(this->cache.pLF_actual.block(0,0,3,1) + this->cache.pRF_actual.block(0,0,3,1))/2.0;
    this->cache.ya  << qb; 
    this->cache.dya << this->robot->dq.segment(BasePosX,6);

    // Comine
    this->cache.eta << this->cache.ya - this->cache.yd, this->cache.dya - this->cache.dyd;
    SymFunction::Dya_standCOM(this->cache.Dya, this->robot->q);
    SymFunction::DLfya_standCOM(this->cache.DLfya, this->robot->q, this->robot->dq);
}

void StandingControlQP::computeDesired(VectorXd &radio) {
    this->cache.yd  << this->config.x_com_offset, -this->lateralFilter.getValue(), -this->heightFilter.getValue(), 0., 0., 0.;
    this->cache.dyd.segment(0,3) << -( this->cache.leftTwist.segment(0,3) + this->cache.rightTwist.segment(0,3) ) / 2.0;
    this->cache.d2yd = (this->cache.dyd - this->memory.dyd_last) / this->config.control_rate;
    for (int i=0; i<6; i++)
        this->cache.d2yd[i] = control_utilities::clamp(this->cache.d2yd[i], -0.75, 0.75);
    this->memory.dyd_last = this->cache.dyd;
    // Hack... lowpass with timing jitter too discontinuous out of Gazebo
    // TODO: come up with better joystick smoothing so we can use the acceleration feedforward term
    this->cache.d2yd.setZero();

    ////////////////////// FOR TESTING \\\\\\\\\\\\\\\\\\\\\\\\
    // The state machine could be cleaned up... but it works fine.
    // Override the crouch with a dynamic and continuous crouch.
    bool triggerCrouch = (-radio(SH) > 0.) && (this->heightFilter.getValue() < 0.99 * this->config.height_lb);

    // Check for start condition
    if (triggerCrouch && !this->memory.crouchOverrideInitialized) {
        this->memory.crouchOverrideCompleted = false;
        this->memory.crouchOverrideTimer.restart();
        this->memory.crouchOverrideInitialized = true;
    }
    // Check for end condition
    if (triggerCrouch && this->memory.crouchOverrideCompleted) {
        this->memory.crouchOverrideCompleted = false;
        this->memory.crouchOverrideTimer.restart();
        this->memory.crouchOverrideInitialized = false;
    }

    double dt = this->memory.crouchOverrideTimer.elapsed();
    if ( this->memory.crouchOverrideInitialized ) {
        double zheight, zvelocity, zaccel;
        if (dt <= 2) {
            zheight = 0.90 - 0.4 / (1+exp(10 - 9*dt));
            zvelocity = -(18*exp(10 - 9*dt))/(5*pow((exp(10 - 9*dt) + 1),2));
            zaccel = (162*exp(10 - 9*dt))/(5*pow((exp(10 - 9*dt) + 1),2)) - (324*exp(20 - 18*dt))/(5*pow((exp(10 - 9*dt) + 1),3));
        } else {
            zheight = 0.5 + 0.4 / (1+exp(26 - 9*dt));
            zvelocity = (18*exp(26 - 9*dt))/(5*pow((exp(26 - 9*dt) + 1),2));
            zaccel = (324*exp(52 - 18*dt))/(5*pow((exp(26 - 9*dt) + 1),3)) - (162*exp(26 - 9*dt))/(5*pow((exp(26 - 9*dt) + 1),2));
        }
        if (dt > 4.) {
            this->memory.crouchOverrideCompleted = true;
            this->memory.crouchOverrideInitialized = false;
        }

        this->cache.yd(2) = zheight;
        this->cache.dyd(2) = zvelocity;
        this->cache.d2yd(2) = zaccel;
    }
}

VectorXd StandingControlQP::getTorqueQP() {
    // Zero torque
    VectorXd u = VectorXd::Zero(10);

    // Formulate the QP
    // States are X = [ddq; u; F; delta] \in \mathbb{R}^{49}
    // Compute the robot constraints
    this->robot->kinematics.update(this->robot->model, this->robot->q, this->robot->dq);
    this->cache.Jc << this->robot->kinematics.cache.J_achilles,
            this->robot->kinematics.cache.J_rigid,
            this->robot->kinematics.cache.J_poseLeftConstraint,
            this->robot->kinematics.cache.J_poseRightConstraint;
    this->cache.dJc << this->robot->kinematics.cache.Jdot_achilles,
            this->robot->kinematics.cache.Jdot_rigid,
            this->robot->kinematics.cache.Jdot_poseLeftConstraint,
            this->robot->kinematics.cache.Jdot_poseRightConstraint;
    this->cache.A_holonomics.block(0,0, 16,22) << this->cache.Jc;
    this->cache.b_holonomics << - this->cache.dJc * this->robot->dq;

    // Compute the robot dynamics via RBDL
    this->robot->dynamics.calcHandC(this->robot->model, this->robot->q, this->robot->dq);
    this->cache.Aeq_dynamics.block(0,0,  22,22) << this->robot->dynamics.H;
    this->cache.Aeq_dynamics.block(0,22, 22,10) << -this->config.Be;
    this->cache.Aeq_dynamics.block(0,32, 22,16) << -this->cache.Jc.transpose();
    this->cache.beq_dynamics << - this->robot->dynamics.C;

    // Torque chatter cost
    this->cache.A_chatter.block(0,22,10,10) << MatrixXd::Identity(10,10);
    this->cache.b_chatter << this->memory.u_prev;

    // Friction cone
    this->cache.A_friction.block(0,38, 9,5) << this->config.contact_pyramid;
    this->cache.A_friction.block(9,43, 9,5) << this->config.contact_pyramid;
    this->cache.b_lb_friction.setConstant(-1e8); // -inf
    this->cache.b_ub_friction.setZero();

    // Task space terms
    this->cache.A_y.block(0,0,6,22) << this->cache.Dya.block(0,0,6,22);
    this->cache.b_y << this->cache.d2yd - this->cache.DLfya.block(0,0,6,22)*this->robot->dq;
    if (config.clf_use_task_pd) {
        this->cache.b_y -= (this->config.Kpy.cwiseProduct(this->cache.eta.segment(0,6)) + this->config.Kdy.cwiseProduct(this->cache.eta.segment(6,6)));
    }

    // Feedforward acceleration from target and constraints
    // \ddot{r}^* = J \ddot{q}^* + \dot{J} \dot{q}
    VectorXd ddq_tar(22), ddr_tar(22);
    ddr_tar.setZero();
    ddr_tar.segment(0,6) << this->cache.d2yd;
    MatrixXd Jddq(22,22), Jdotddq(22,22);
    Jddq << this->cache.Dya.block(0,0,6,22),
            this->cache.Jc;
    Jdotddq << this->cache.DLfya.block(0,0,6,22),
            this->cache.dJc;
    ddq_tar = Jddq.completeOrthogonalDecomposition().solve(ddr_tar - Jdotddq * this->robot->dq);

    // CLF terms
    this->cache.V = this->cache.eta.transpose() * this->config.P * this->cache.eta;
    double LFV = this->cache.eta.transpose() * this->config.LFV_mat * this->cache.eta;
    MatrixXd LGV(1,6);
    LGV << 2. * this->cache.eta.transpose() * this->config.P * this->config.G;
    double LGVpsi1 = (LGV* (this->cache.DLfya.block(0,0,6,22) * this->robot->dq - this->cache.d2yd)).value();

    // CLF Constraint as inequality
    this->cache.A_clf.block(0,0,1,22) = LGV * this->cache.Dya.block(0,0,6,22);
    this->cache.A_clf(0,48) = -1.;
    double b_lb_clf = -1e12;
    double b_ub_clf =  1e12;
    if (this->config.clf_use_inequality)
        b_ub_clf = - this->config.gam * this->cache.V - LFV - LGVpsi1;

    // Build all constraint matrices
    VectorXd lb(49), ub(49);
    lb << -1e10 * VectorXd::Ones(22),           // ddq
            -this->config.torque_bounds,        // torque
            -10000., -10000.,                   // f 2x achilles,
            -10000., -10000., -10000., -10000., // f 4x rigid
            -400., -500., 0., -300., -300.,     // f Left Foot
            -400., -500., 0., -300., -300.,     // f Right Foot
            0;                                  // qp relaxation delta
    ub << 1e10 * VectorXd::Ones(22),        // ddq
            this->config.torque_bounds,     // torque
            10000., 10000.,                 // f 2x achilles,
            10000., 10000., 10000., 10000., // f 4x rigid
            400., 500., 1200., 300., 300.,  // f Left Foot
            400., 500., 1200., 300., 300.,  // f Right Foot
            1e6;                            // qp relaxation delta

    Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> Aconstr(this->cache.Aeq_dynamics.rows()+this->cache.A_friction.rows()+1, 49);
    Aconstr << this->cache.Aeq_dynamics,
            this->cache.A_friction,
            this->cache.A_clf;
    VectorXd lbAconstr(this->cache.Aeq_dynamics.rows()+this->cache.A_friction.rows()+1), ubAconstr(this->cache.Aeq_dynamics.rows()+this->cache.A_friction.rows()+1);
    lbAconstr << this->cache.beq_dynamics,
            this->cache.b_lb_friction,
            b_lb_clf;
    ubAconstr << this->cache.beq_dynamics,
            this->cache.b_ub_friction,
            b_ub_clf;

    // Construct the cost
    // 0.5*x'*H*x + f'*x
    // Cost weighting terms
    MatrixXd A_reg = MatrixXd::Identity(49,49);
    VectorXd w_reg(49), b_reg(49);
    b_reg.setZero();
    b_reg.segment(0,22) << ddq_tar;
    b_reg(41) = 150.;
    b_reg(46) = 150.;

    w_reg << this->config.reg_ddq * VectorXd::Ones(22),    // ddq
            this->config.reg_u * VectorXd::Ones(10),       // u
            this->config.reg_achilles * VectorXd::Ones(2), // lambda_ach
            this->config.reg_rigid * VectorXd::Ones(4),    // lambda_rigid
            this->config.reg_fx,        // fx_left
            this->config.reg_fy,        // fy_left
            this->config.reg_fz,        // fz_left
            this->config.reg_muy,       // mu_y_left
            this->config.reg_muz,       // mu_z_left
            this->config.reg_fx,        // fx_right
            this->config.reg_fy,        // fy_right
            this->config.reg_fz,        // fz_right
            this->config.reg_muy,       // mu_y_right
            this->config.reg_muz,       // mu_z_right
            this->config.reg_clf_delta; // clf delta

    VectorXd w_u_chatter(10);
    w_u_chatter << this->config.w_u_chatter * VectorXd::Ones(10);

    VectorXd w_hol(16);
    VectorXd w_con(5);
    w_con << this->config.w_hol_fx, this->config.w_hol_fy, this->config.w_hol_fz,
            this->config.w_hol_my, this->config.w_hol_mz;
    w_hol << this->config.w_hol_achilles * VectorXd::Ones(2), // achilles
            this->config.w_hol_fixed * VectorXd::Ones(4),     // rigid spring
            w_con,                                            // contact left
            w_con;                                            // contact right

    // Build the final QP formulation
    long n_var = 49;
    long n_cost_terms = w_hol.size() + w_u_chatter.size() + A_reg.rows() + 6; //+ w_reg.size()
    VectorXd w(n_cost_terms);
    Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> A(n_cost_terms,n_var); // Gets passed directly to
    VectorXd b(n_cost_terms);
    w << w_hol,
            w_u_chatter,
            this->config.w_outputs * VectorXd::Ones(6),
            w_reg;
    A << this->cache.A_holonomics,
            this->cache.A_chatter,
            this->cache.A_y,
            A_reg;
    b << this->cache.b_holonomics,
            this->cache.b_chatter,
            this->cache.b_y,
            b_reg;
    for (int i=0; i<w.size(); i++) {
        b(i) *= w(i);
        A.row(i) *= w(i);
    }

    // Need to declare row major for all matrices getting mapped to
    // normal real_t arrays in qpoases.
    Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> G(49,49);
    VectorXd g(49);
    G << A.transpose() * A;
    //G.diagonal() += w_reg;
    g << -A.transpose() * b;
    //g.segment(0,22) -= ddq_tar.cwiseProduct(w_reg.segment(0,22));

    if (this->config.clf_use_Vdot_cost)
        g.segment(0,22) += this->config.w_Vdot * (LGV * this->cache.Dya.block(0,0,6,22)).transpose();

    // Flatten matrices in row-major format.
    Map<VectorXd> Gflat(G.data(), G.size());
    Map<VectorXd> AConstrFlat(Aconstr.data(), Aconstr.size());

    // Run the QP
    qpOASES::returnValue success = RET_QP_NOT_SOLVED;
    int_t nWSR = this->config.nQPIter;
    if (this->memory.qp_initialized) {
        success = this->qpsolver->hotstart(static_cast<real_t*>(Gflat.data()), static_cast<real_t*>(g.data()),
                                 static_cast<real_t*>(AConstrFlat.data()),
                                 static_cast<real_t*>(lb.data()), static_cast<real_t*>(ub.data()),
                                 static_cast<real_t*>(lbAconstr.data()),static_cast<real_t*>(ubAconstr.data()),
                                 nWSR );
    } else {
        success = this->qpsolver->init( static_cast<real_t*>(Gflat.data()), static_cast<real_t*>(g.data()),
                              static_cast<real_t*>(AConstrFlat.data()),
                              static_cast<real_t*>(lb.data()), static_cast<real_t*>(ub.data()),
                              static_cast<real_t*>(lbAconstr.data()),static_cast<real_t*>(ubAconstr.data()),
                              nWSR );
        this->memory.qp_initialized = true;
    }


    // Get solution
    VectorXd torqueScale(10);
    torqueScale << 25., 25., 16., 16., 50., 25., 25., 16., 16., 50.;
    this->cache.qpsol.setZero();
    this->qpsolver->getPrimalSolution(static_cast<real_t*>(this->cache.qpsol.data()));
    if (success != SUCCESSFUL_RETURN) {
        ROS_WARN("THE QP DID NOT CONVERGE!");
        this->runInverseKinematics(this->cache.pLF_actual, this->cache.pRF_actual);
        u = this->getTorqueID();
        this->qpsolver->reset();
        this->memory.qp_initialized = false;
        this->memory.u_prev.setZero();
    }
    else {
        this->memory.u_prev = this->cache.qpsol.segment(22,10);
        u << torqueScale.cwiseProduct(this->cache.qpsol.segment(22,10));
    }
    this->cache.Fdes << this->cache.qpsol.segment(32,16);
    this->cache.delta = this->cache.qpsol(48);

    // Return
    this->cache.u << u;
    return u;
}

VectorXd StandingControlQP::getTorqueID() {
    VectorXd u(10);
    VectorXd q_motors(10), dq_motors(10);
    for (int i = 0; i < this->robot->iRotorMap.size(); ++i) {
        int ind = this->robot->iRotorMap(i);
        q_motors(i) = this->robot->q(ind);
        dq_motors(i) = this->robot->dq(ind);
    }

    // Compute torques via PD
    u = this->pd.compute(this->cache.IK_solution, this->cache.dq_desired, q_motors, dq_motors);

    if (this->memory.contact_initialized) {
        // Feedforward from Inverse Dynamics
        this->robot->dynamics.calcHandC(this->robot->model, this->robot->q, this->robot->dq);
        MatrixXd B(22,10);
        B.setZero();
        B(LeftHipRoll,0) = 1.;
        B(LeftHipYaw,1) = 1.;
        B(LeftHipPitch,2) = 1.;
        B(LeftKneePitch,3) = 1.;
        B(LeftFootPitch,4) = 1.;
        B(RightHipRoll,5) = 1.;
        B(RightHipYaw,6) = 1.;
        B(RightHipPitch,7) = 1.;
        B(RightKneePitch,8) = 1.;
        B(RightFootPitch,9) = 1.;

        // Compute the robot constraints
        MatrixXd Jc(14,22);
        this->robot->kinematics.update(this->robot->model, this->robot->q, this->robot->q);
        Jc << this->robot->kinematics.cache.J_achilles,
                this->robot->kinematics.cache.J_rigid,
                this->robot->kinematics.cache.J_poseLeftConstraint.block(0,0,4,22), // Leave out foot yaw
                this->robot->kinematics.cache.J_poseRightConstraint.block(0,0,4,22);

        MatrixXd Ge(22,1); Ge.setZero();
        SymFunction::Ge_cassie_v4(Ge, this->robot->q);

        MatrixXd P(22,22);
        P = MatrixXd::Identity(22,22) - Jc.completeOrthogonalDecomposition().solve(Jc);

        VectorXd ff(10);
        MatrixXd PB(22,10);
        PB = P*B;
        //ff = PB.completeOrthogonalDecomposition().solve(P) * Ge;
        ff = PB.completeOrthogonalDecomposition().solve(P) * this->robot->dynamics.C;
        u += ff;

        // Add a naiive toe regulator...
        u(4) -= 45. * (this->robot->q(BaseRotY) - this->cache.pitch_desired) + 2. * this->robot->dq(BaseRotY);
        u(9) -= 45. * (this->robot->q(BaseRotY) - this->cache.pitch_desired) + 2. * this->robot->dq(BaseRotY);
    }
    return u;
}

void StandingControlQP::runInverseKinematics(MatrixXd &pLF, MatrixXd &pRF) {
    // Get desired pelvis position
    double pdes = this->pitchFilter.getValue();
    Matrix3d Rot;
    Rot << cos(pdes), 0, sin(pdes),
           0, 1, 0,
           -sin(pdes), 0, cos(pdes);
    this->cache.T_des_pelvis.matrix().block(0,0,3,3) << Rot;

    VectorXd footDiff(3);
    footDiff = this->memory.T_leftContact.translation() - this->memory.T_rightContact.translation();

    // X position
    this->cache.T_des_leftFoot(0,3)  = -footDiff(0) - this->cache.yd(0);
    this->cache.T_des_rightFoot(0,3) =  footDiff(0) - this->cache.yd(0);

    // Y position
    this->cache.T_des_leftFoot(1,3)  =  footDiff(1) / 2. - this->cache.yd(1);
    this->cache.T_des_rightFoot(1,3) = -footDiff(1) / 2. - this->cache.yd(1);

    // Z position
    double roll  = this->robot->q(3);
    double droll = this->robot->dq(3);
    double lateralStabilizer = 0.0;
    if (this->config.use_lateral_comp)
        lateralStabilizer = this->memory.spoolup * (this->config.Kp_lateral_compensator * roll + this->config.Kd_lateral_compensator * droll);
    this->cache.T_des_leftFoot(2,3)  = -this->cache.yd(2)  + lateralStabilizer / 2.0;
    this->cache.T_des_rightFoot(2,3) = -this->cache.yd(2) - lateralStabilizer / 2.0;

    // Set residual function target
    double footpitch_target = 0.0;
    this->IKfunction.setTarget(this->cache.T_des_pelvis,
                               this->cache.T_des_leftFoot, footpitch_target,
                               this->cache.T_des_rightFoot, footpitch_target);

    // Solve the IK problem - uses current state as the initial guess
    int iksuccess = this->InverseKinematics(this->cache.IK_solution);
    if (iksuccess < 0) {
        // Don't use solution if it failed ...
        // Just do what the last command was
        this->cache.IK_solution << this->memory.IK_solution_prev;
    }

    // Compute the desired velocities
    this->cache.target_velocities << this->cache.dyd.segment(0,3),  0.0, -this->cache.dyd(4),
                                     this->cache.dyd.segment(0,3),  0.0, -this->cache.dyd(4);

    this->IKfunction.df(this->cache.IK_solution, this->cache.Jik);
    this->cache.dq_desired = this->cache.Jik.inverse() * this->cache.target_velocities;
}

void StandingControlQP::setFootTransforms() {
    // Set the current foot transforms based on the robot state
    VectorXd q(22);
    q << this->robot->q;
    q.segment(BasePosX,6) << 0,0,0,0,0,0;
    q(LeftShinPitch)    = 0.0;
    q(LeftTarsusPitch)  = 0.226893 - q(LeftKneePitch); // 13 deg - knee
    q(RightShinPitch)   = 0.0;
    q(RightTarsusPitch) = 0.226893 - q(RightKneePitch);

    MatrixXd p_leftFoot(5,1), p_rightFoot(5,1);
    SymFunction::p_leftSole_constraint(p_leftFoot, q);
    SymFunction::p_rightSole_constraint(p_rightFoot, q);

    this->memory.T_leftContact.translation()  = p_leftFoot.block(0,0,3,1);
    this->memory.T_rightContact.translation() = p_rightFoot.block(0,0,3,1);

    // Set filters for X
    this->leftXFilter.update(p_leftFoot(0));
    this->rightXFilter.update(p_rightFoot(0));

    // Lateral filter
    this->lateralFilter.update( (p_leftFoot(1) + p_rightFoot(1)) / 2.0 );

    // Set the desired heights
    this->heightFilter.update(p_leftFoot(2));
    this->heightFilter.update(p_rightFoot(2));

    // Pitch filter
    this->pitchFilter.update( q(BaseRotY) );

    std::cout << "LeftContact registered at: \n\t" << this->memory.T_leftContact.translation().transpose() << std::endl;
    std::cout << "RightContact registered at: \n\t" << this->memory.T_rightContact.translation().transpose() << std::endl;
}

void StandingControlQP::processRadio(VectorXd &radio) {
    // Stored values
    double last_left_x = this->leftXFilter.getValue();
    double last_right_x = this->rightXFilter.getValue();
    double last_lateral = this->lateralFilter.getValue();
    double last_left_height = this->heightFilter.getValue();
    double last_right_height = this->heightFilter.getValue();
    double last_pitch = this->pitchFilter.getValue();

    // Check for transition request
    if (!this->memory.queueTransition && (radio(SB) > 0.1)) {
        // Radio was moved up, queue the transition
        ROS_INFO("Requested a transition to walking!");
        this->memory.queueTransition = true;
    }
    // If we have a queue stored, overwrite the radio commands to do the desired motion to walking
    if (this->memory.queueTransition) {
        // Spoof the radio commands
        radio(RV) = 0.0;
        radio(LS) = 1.0;
        radio(RH) = 1.0;
    }

    // X
    double x_width = this->memory.T_leftContact.translation()[0] - this->memory.T_rightContact.translation()[0];
    double x_desired = 0;
    if (x_width > 0) {
        // left foot forward
        this->leftXFilter.update(x_desired - x_width/2.0);
        this->rightXFilter.update(x_desired + x_width/2.0);
    } else {
        // right foot forward
        this->leftXFilter.update(x_desired + x_width/2.0);
        this->rightXFilter.update(x_desired - x_width/2.0);
    }
    this->cache.leftTwist(0) = (this->leftXFilter.getValue() - last_left_x) / this->config.control_rate;
    this->cache.rightTwist(0) = (this->rightXFilter.getValue() - last_right_x) / this->config.control_rate;

    // Y
    double lateral_width = this->memory.T_leftContact.translation()[1] - this->memory.T_rightContact.translation()[1];
    double latRange_lb = this->config.lateral_lb * lateral_width/2.0; // % of the recorded width
    double latRange_ub = this->config.lateral_ub * lateral_width/2.0;
    double latDesiredRaw = radio(RadioButtonMap::LH);
    latDesiredRaw = lateral_width * latDesiredRaw * (latRange_ub - latRange_lb) / 2.0;

    this->lateralFilter.update((latDesiredRaw + lateral_width)/2.0);
    this->lateralFilter.update((latDesiredRaw - lateral_width)/2.0);

    double leftLateralDesired = this->lateralFilter.getValue();
    double rightLateralDesired = this->lateralFilter.getValue();
    this->cache.leftTwist(1)  = (leftLateralDesired - last_lateral) / this->config.control_rate;
    this->cache.rightTwist(1) = (rightLateralDesired - last_lateral) / this->config.control_rate;

    // Z
    double hRange_lb = this->config.height_lb;
    double hRange_ub = this->config.height_ub;
    double zDesiredRaw = (1.0 + radio(RadioButtonMap::LS)) / 2.0;
    zDesiredRaw = hRange_ub - zDesiredRaw * (hRange_ub - hRange_lb);
    this->heightFilter.update(zDesiredRaw);
    this->heightFilter.update(zDesiredRaw);

    this->cache.leftTwist(2) = ( this->heightFilter.getValue() - last_left_height ) / this->config.control_rate;
    this->cache.rightTwist(2) = ( this->heightFilter.getValue() - last_right_height ) / this->config.control_rate;

    // Pitch
    double pRange_lb = this->config.pitch_lb;
    double pRange_ub = this->config.pitch_ub;
    double pDesiredRaw = radio(RadioButtonMap::RV);
    pDesiredRaw = pDesiredRaw * (pRange_ub - pRange_lb) / 2.0;
    this->pitchFilter.update(pDesiredRaw);

    this->cache.pitch_desired = this->pitchFilter.getValue();
    this->cache.leftTwist(3) = ( this->cache.pitch_desired - last_pitch ) / this->config.control_rate;
    this->cache.rightTwist(3) = ( this->cache.pitch_desired - last_pitch ) / this->config.control_rate;

    // Use the pitch rate to update the twist for the leg cartesian velocitites
    //Vector3d w(0.0, this->cache.leftTwist(3), 0.0);
    //this->cache.leftTwist.block(0,0,3,1) = skew(w) * this->cache.leftTwist.block(0,0,3,1);
    //this->cache.rightTwist.block(0,0,3,1) = skew(w) * this->cache.rightTwist.block(0,0,3,1);

    /*
    // Turn off chatter?
    for (int i=0; i<4; ++i) {
        if (fabs(this->cache.leftTwist(i)) < 0.05)
            this->cache.leftTwist(i) = 0;
        if (fabs(this->cache.rightTwist(i)) < 0.05)
            this->cache.rightTwist(i) = 0;
    }
    */
}

void StandingControlQP::getDebug(VectorXf &dbg) {
    dbg << static_cast<float>(ros::Time::now().toSec()), // 1
            this->cache.ya.cast <float> (),    // 6
            this->cache.dya.cast <float> (),   // 6
            this->cache.yd.cast <float> (),    // 6
            this->cache.dyd.cast <float> (),   // 6
            this->cache.d2yd.cast <float> (),  // 6
            static_cast<float>(this->cache.V), // 1
            this->cache.u.cast <float> (),     // 10
            this->cache.Fdes.cast <float> (),  // 16
            static_cast<float>(this->cache.delta); // 1
    // ndbg = 59
}

int StandingControlQP::InverseKinematics(VectorXd &x) {
    int iteration_limit = this->config.ik_iter_limit;
    double xtol = this->config.ik_xtol;
    VectorXd q(22);
    q << robot->q;
    for (int i=0; i<6; ++i)
        q(i) = 0.0;
    q(LeftShinPitch)  = 0.0;
    q(RightShinPitch) = 0.0;

    VectorXd F(10);
    MatrixXd J(10,10);
    double f_d = 0.0;

    for (int i=0; i<iteration_limit; ++i){
        // Reinitialize the state at the current guess
        this->IKfunction(x, F);

        // Check for completion
        f_d = 0.5 * F.transpose() * F;
        if (f_d < xtol) {
            return i;
        }

        // Compute the Jacobian
        this->IKfunction.df(x, J);

        // Perform the update
        x = x - (J.transpose() * (J * J.transpose()).inverse()) * F;
    }
    ROS_WARN("IK DID NOT CONVERGE");
    return -1;
}

