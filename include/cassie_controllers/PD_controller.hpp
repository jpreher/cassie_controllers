/**
    @author Jenna Reher (jreher@caltech.edu)
*/

#ifndef CASSIE_PD_CONTROLLER_HPP
#define CASSIE_PD_CONTROLLER_HPP

#include <Eigen/Dense>
#include <eigen_utilities/assert_size.hpp>
#include <iostream>

class PD_Controller {
public:
    Eigen::VectorXd Kp;
    Eigen::VectorXd Kd;
    Eigen::VectorXi zero_indices; // TODO
    Eigen::VectorXd u;

    PD_Controller() {
        this->isInitialized = false;
    }

    PD_Controller(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd) {
        this->reconfigure(Kp, Kd);
    }

    void reconfigure(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd) {
        eigen_utilities::assert_size(Kp, Kd.size());
        this->Kp.resize(Kp.size());
        this->Kd.resize(Kd.size());
        this->Kp << Kp;
        this->Kd << Kd;
        this->u.setZero(Kp.size());
        this->isInitialized = true;
    }

    Eigen::VectorXd compute(Eigen::VectorXd &q_desired, Eigen::VectorXd &dq_desired, Eigen::VectorXd &q_actual, Eigen::VectorXd &dq_actual) {
        // Cannot run if not initialized
        assert(this->isInitialized);

        // Size check everything!
        eigen_utilities::assert_size(q_desired, this->Kd.size());
        eigen_utilities::assert_size(dq_desired, this->Kd.size());
        eigen_utilities::assert_size(q_actual, this->Kd.size());
        eigen_utilities::assert_size(dq_actual, this->Kd.size());

        // Compute and return the vector
        this->u << - (this->Kp.cwiseProduct(q_actual - q_desired) + this->Kd.cwiseProduct(dq_actual - dq_desired));
        return this->u;
    }
private:
    bool isInitialized;
};


#endif // CASSIE_PD_CONTROLLER_HPP
