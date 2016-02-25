
#include "orca_afq/SimulatorNode.hpp"
#include "ros/ros.h"

#define RATE 30

int main(int argc, char **argv) {

    ros::init(argc, argv, "simulator");

    Simulation::SimulatorNode node(RATE);
    ros::Rate loop_rate(RATE);

    while (ros::ok()) {
        node.doStep(); // computes best velocities and sends to physical simulation
        ros::spinOnce(); // updates the inner simulation with estimated real pos/vel
        loop_rate.sleep(); // rate control
    }

    return 0;
}
