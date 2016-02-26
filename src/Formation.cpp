#include "orca_afq/Formation.hpp"

namespace Simulation {
    void Formation::addAgent(size_t agentNo, RVO::Vector3 slot) {
        slots_[agentNo] = slot;
    }

    void Formation::updateTargets() {

        // If bound to a leader, update pos/vel accordingly
        if (leaderNo_ >= 0) {
            position_ = sim_node_->rvo_sim_.getAgentPosition(leaderNo_);
            velocity_ = sim_node_->rvo_sim_.getAgentVelocity(leaderNo_);
        }

        // Update agents' targets
        for (auto const& slot : slots_) {
            sim_node_->agents_[slot.first].setTarget(position_ + slot.second, velocity_);
        }
    }
}
