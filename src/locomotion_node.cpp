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

// ROS
#include <ros/ros.h>

// Robot descriptions
#include <cassie_description/cassie_model.hpp>
#include <cassie_common_toolbox/RadioButtonMap.hpp>

// Controllers
#include <cassie_controllers/standing_control_qp.hpp>
#include <cassie_controllers/walking_onedomain.hpp>

// Load in message types
#include <cassie_common_toolbox/callback_helpers.hpp>
#include <cassie_common_toolbox/cassie_control_msg.h>
#include <cassie_common_toolbox/cassie_proprioception_msg.h>

#include <realtime_utilities/timing.hpp>

// Logging
#include <fstream>
#include <cmath>

using namespace Eigen;
using namespace cassie_model;

// Robotic states
static bool is_initialized = false;
static Cassie robot;
static VectorXd radio(16);
static int mode_command = -1; // Coming from the radio

// Callback for proprioception subscriber
void proprioception_callback(const cassie_common_toolbox::cassie_proprioception_msg::ConstPtr& propmsg)
{
    robot.q.setZero();
    robot.dq.setZero();
    unpack_proprioception(propmsg, robot.q, robot.dq, radio, robot.gyroscope, robot.accelerometer, robot.torque, robot.leftContact, robot.rightContact);
    get_proprioception_orientation(*propmsg, robot.q, robot.dq, robot.quat_pelvis);
    mode_command = (int)radio(RadioButtonMap::SB);
    is_initialized = true;
}

// Main node
int main(int argc, char *argv[])
{
    // Establish the current ROS node and associated timing
    ros::init(argc, argv, "locomotion_control");
    ros::NodeHandle nh("/cassie/locomotion_control");

    // Setup ROS publisher/subscriber networks
    double dt_des;
    ros::param::get("/cassie/locomotion_control/dt", dt_des);
    ros::Rate looprate(1/dt_des); // Run at 2 kHz
    ros::Publisher  controller_pub = nh.advertise<cassie_common_toolbox::cassie_control_msg>("/cassie_control", 1);
    ros::Subscriber proprioception_sub = nh.subscribe("/cassie_proprioception", 1, proprioception_callback, ros::TransportHints().tcpNoDelay(true));
    cassie_common_toolbox::cassie_control_msg control_message;

    // Create the associated controllers
    StandingControlQP stand_control(nh, robot);
    Walking1DControl walk_control(nh, robot);

    // Logging things
    VectorXf qplog(82); qplog.setZero();//+44
    VectorXf pdlog(60); pdlog.setZero();
    VectorXf standlog(60); standlog.setZero();
    std::fstream logfileStand, logfileWalk;
    bool log_controller = false;
    ros::param::get("/cassie/log_controller", log_controller);
    if ( log_controller ) {
        std::string home=getenv("HOME");

        {
        std::string path= home+"/datalog/qp_walk_log.bin";
        logfileWalk.open(path, std::ios::out | std::ios::binary);
        }
        {
        std::string path= home+"/datalog/stand_log.bin";
        logfileStand.open(path, std::ios::out | std::ios::binary);
        }
    }

    // Listen/respond loop
    int mode = -1;
    VectorXd u(10); u.setZero();
    while (ros::ok()) {
        // Do things
        ros::spinOnce();

        // Zero the torque and set the message timestamp
        control_message.motor_torque.fill(0.0);

        // Main state machine
        if (is_initialized) {

            switch (mode) {
            case -1 : {
                for (unsigned int i = 0; i<10; ++i) {
                    control_message.motor_torque[i] = 0.0;
                }

                if (mode_command > -1) {
                    ROS_INFO("Transitioning to standing control!");
                    stand_control.reset();
                    mode = 0;
                }
                break;
            }
            case 0 : {
                stand_control.update(radio, u);
                for (unsigned int i = 0; i<10; ++i) {
                    control_message.motor_torque[i] = u(i);
                }
                if ( log_controller ) {
                    stand_control.getDebug(standlog);
                    logfileStand.write(reinterpret_cast<char *>(standlog.data()), (standlog.size())*sizeof(float));
                }

                if (mode_command < 0) {
                    ROS_INFO("Transitioning to null control!");
                    mode = -1;
                }
                if ( (mode_command > 0) && (stand_control.isReadyToTransition()) ) {
                    ROS_INFO("Transitioning to stepping control!");
                    walk_control.reset();
                    mode = 1;
                    stand_control.reset();
                }
                break;
            }
            case 1: {
                walk_control.update(radio, u);

                for (unsigned int i = 0; i<10; ++i) {
                    control_message.motor_torque[i] = u(i);
                }

                if ( log_controller ) {
                    walk_control.getDebug(qplog);

                    logfileWalk.write(reinterpret_cast<char *>(qplog.data()), (qplog.size())*sizeof(float));
                }

                if ( (mode_command == 0) && ( walk_control.isReadyToTransition() ) ) {
                    ROS_INFO("Transitioning to standing control!");
                    walk_control.reset();
                    stand_control.reset();
                    mode = 0;
                }
                if (mode_command == -1) {
                    ROS_INFO("Transitioning to null control!");
                    mode = -1;
                }
                break;
            }
            default :
                ROS_ERROR("The controller state machine done broke itself...");
                return 1;
            }
        }

        for (unsigned int i = 0; i<10; ++i) {
            bool val = std::isnan(control_message.motor_torque[i]);
            if ( val ) {
                control_message.motor_torque.fill(0.0);
                control_message.header.stamp = ros::Time::now();
                controller_pub.publish(control_message);
                ROS_ERROR("The controller generated a nan torque!!!!!!!");
                return 1;
            }
        }

        // Publish the control message
        control_message.header.stamp = ros::Time::now();
        controller_pub.publish(control_message);

        looprate.sleep();
    }

    if ( log_controller ) {
        logfileWalk.close();
        logfileStand.close();
    }

    return 0;
}
