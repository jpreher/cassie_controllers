/**
    @author Jenna Reher (jreher@caltech.edu)
*/

#ifndef CASSIE_IK_HPP
#define CASSIE_IK_HPP

#include <cassie_description/cassie_model.hpp>

namespace cassie_inverse_kinematics {

struct StandingInverseKinematics
{
    // Objective targets
    cassie_model::Cassie *robot;
    Eigen::VectorXd target;
    int m = 10; // Numer of target functions
    int n = 10; // The number of parameters, i.e. inputs.

    StandingInverseKinematics();
    void init(cassie_model::Cassie &robot);

    // Compute 'm' errors, one for each target, for the given parameter values in 'x'
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

    // Compute the jacobian of the errors
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const;

    void setTarget(Eigen::Transform<double, 3, Affine> &T_pelvis, Eigen::Transform<double, 3, Affine> &T_lf, double pitchLF, Eigen::Transform<double, 3, Affine> &T_rf, double pitchRF);
};

}

#endif // CASSIE_IK_HPP
