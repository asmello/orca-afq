#include "orca_afq/SimulatorNode.hpp"

namespace Simulation {
    SimulatorNode::SimulatorNode(float rate) {

        rvo_sim_.setTimeStep(1.0f/rate);
        rvo_sim_.setAgentDefaults(15.0f, 10, 2.0f, 0.5f, 1.0f); // change

        // Add agents to simulation
        agents_.emplace_back(this, RVO::Vector3(0.0f, 1.0f, 0.5f), "uav1");
        agents_.emplace_back(this, RVO::Vector3(0.0f, -1.0f, 0.5f), "uav2");

        for (Agent& a : agents_) {
            a.init();
            a.setTarget(RVO::Vector3(-2.5f, 5.0f, 10.0f)); // static target
        }
    }

    void SimulatorNode::doStep() {

        // Set preferred velocities in ORCA simulator
        for (Agent& a : agents_) {
            rvo_sim_.setAgentPrefVelocity(a.getNumber(), a.computePrefVelocity());
        }

        rvo_sim_.doStep(); // Advance time inside the ORCA simulator

        // Move agents in the physical simulation
        for (const Agent& a : agents_) {
            a.sendCmd();
        }
    }
}
