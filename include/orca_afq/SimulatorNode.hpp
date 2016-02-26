#ifndef SIMULATOR_NODE_HPP
#define SIMULATOR_NODE_HPP

#include "rvo2_3d/RVO.h"

#include "ros/ros.h"
#include "orca_afq/Agent.hpp"
#include "orca_afq/Formation.hpp"

#include <vector>

namespace Simulation {
    class Agent;
    class Formation;

    class SimulatorNode {
    private:
        RVO::RVOSimulator rvo_sim_;
        ros::NodeHandle node_handle_;
        std::vector<Agent> agents_;
        std::vector<Formation> formations_;

    public:
        SimulatorNode(float rate);
        void doStep();

        friend class Agent;
        friend class Formation;
    };
}

#endif
