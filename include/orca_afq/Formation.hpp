#ifndef FORMATION_HPP
#define FORMATION_HPP

#include "rvo2_3d/RVO.h"
#include "orca_afq/SimulatorNode.hpp"

#include <unordered_map>

namespace Simulation {
    class SimulatorNode;

    class Formation {
    private:
        std::unordered_map<size_t, RVO::Vector3> slots_;
        RVO::Vector3 position_, velocity_;
        SimulatorNode *sim_node_;
        size_t leaderNo_;

    public:
        Formation(SimulatorNode *node, RVO::Vector3 position=RVO::Vector3(), RVO::Vector3 velocity=RVO::Vector3())
        : sim_node_(node), position_(position), velocity_(velocity), leaderNo_(-1) {}

        void addAgent(size_t agentNo, RVO::Vector3 slot);
        void updateTargets();
        void setVelocity(RVO::Vector3 velocity) { velocity_ = velocity; }
        void setPosition(RVO::Vector3 position) { position_ = position; }
        void setLeader(size_t agentNo) { leaderNo_ = agentNo; }
    };
}

#endif
