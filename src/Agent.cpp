#include "orca_afq/Agent.hpp"

#include "rvo2_3d/Definitions.h"
#include "geometry_msgs/Twist.h"

namespace Simulation {
    Agent::Agent(SimulatorNode *node, RVO::Vector3 position, std::string ns)
    : sim_node_(node), namespace_(ns), ini_position_(position) { }

    // Necessary because the "this" pointer might not be final upon construction
    void Agent::init() {
        velocity_publisher_ = sim_node_->node_handle_.advertise<geometry_msgs::Twist>(namespace_ + "/cmd_vel", 1000);
        velocity_subscriber_ = sim_node_->node_handle_.subscribe<geometry_msgs::Vector3Stamped>(namespace_ + "/velocity", 1, &Agent::velocity_callback, this);
        pose_subscriber_ = sim_node_->node_handle_.subscribe<geometry_msgs::PoseStamped>(namespace_ + "/ground_truth_to_tf/pose", 1, &Agent::pose_callback, this);
        agentNo_ = sim_node_->rvo_sim_.addAgent(ini_position_);
    }

    // Updates the position of the agent (from physical simulation)
    void Agent::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
        RVO::Vector3 position(data->pose.position.x, data->pose.position.y, data->pose.position.z);
        sim_node_->rvo_sim_.setAgentPosition(agentNo_, position);
    }

    // Updates the velocity of the agent (from physical simulation)
    void Agent::velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& data) {
        RVO::Vector3 velocity(data->vector.x, data->vector.y, data->vector.z);
        sim_node_->rvo_sim_.setAgentVelocity(agentNo_, velocity);
    }

    void Agent::setTarget(RVO::Vector3 position, RVO::Vector3 velocity) {
		targetPosition_ = position;
		targetVelocity_ = velocity;
	}

	RVO::Vector3 Agent::computePrefVelocity() const {
        const float approxRate = 1.0f; // tweakable

		// Estimate target's future pose
		const RVO::Vector3 targetFuturePos = targetPosition_ + targetVelocity_ * sim_node_->rvo_sim_.getAgentTimeHorizon(agentNo_);
		const RVO::Vector3 targetFutureVel = targetVelocity_;
		// TODO: use a better estimation

		const  RVO::Vector3 relativePosition = targetFuturePos - sim_node_->rvo_sim_.getAgentPosition(agentNo_);
		const float distSq = RVO::absSq(relativePosition);
		const float sigmaSq = RVO::sqr(approxRate);

		// Benzerrouk's attraction law
        const float maxSpeed = sim_node_->rvo_sim_.getAgentMaxSpeed(agentNo_);
		const float speed = maxSpeed - (maxSpeed - abs(targetFutureVel)) * std::exp(-distSq/sigmaSq);

		return RVO::normalize(relativePosition) * speed;
	}

    void Agent::sendCmd() const {
        RVO::Vector3 vel = sim_node_->rvo_sim_.getAgentVelocity(agentNo_);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = vel.x();
        cmd_vel.linear.y = vel.y();
        cmd_vel.linear.z = vel.z();
        velocity_publisher_.publish(cmd_vel);
    }
}
