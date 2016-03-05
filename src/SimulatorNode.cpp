#include "orca_afq/SimulatorNode.hpp"

namespace Simulation {
    SimulatorNode::SimulatorNode(float rate) {

        rvo_sim_.setTimeStep(1.0f/rate);
        rvo_sim_.setAgentDefaults(20.0f, 10, 1.5f, 1.0f, 1.25f); // change

        // Add agents to simulation
        agents_.emplace_back(this, RVO::Vector3(1.0f, 1.5f, 0.5f), "uav1");
        agents_.emplace_back(this, RVO::Vector3(-1.0f, -1.5, 0.5f), "uav2");
        agents_.emplace_back(this, RVO::Vector3(0.0f, -15.0f, 1.0f), "uav3");

        formations_.emplace_back(this); // Create a new formation
        formations_[0].setLeader(2); // And bind it to agent 3

        // Add the other agents to the formation (so it controls their targets)
        formations_[0].addAgent(0, RVO::Vector3(3.0f, -2.0f, 3.0f));
        formations_[0].addAgent(1, RVO::Vector3(-3.0f, -2.0f, 3.0f));

        // Manually set agent 3's target
        agents_[2].setTarget(RVO::Vector3(0.0f, 20.0f, 20.0f));

        // Initialize agent state (ROS topics, ORCA inner representation)
        for (Agent& a : agents_) a.init();

        // Make the leader slower so the other can catch up
        rvo_sim_.setAgentMaxSpeed(2, 0.6f);
    }

    void SimulatorNode::doStep() {

        for (Formation& formation : formations_) {
            formation.updateTargets();
        }

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
