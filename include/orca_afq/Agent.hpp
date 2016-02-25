#ifndef AGENT_HPP
#define AGENT_HPP

#include "rvo2_3d/RVO.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "orca_afq/SimulatorNode.hpp"

#include <string>

namespace Simulation {
    class SimulatorNode;

    class Agent {
    private:
        ros::Subscriber pose_subscriber_;
        ros::Subscriber velocity_subscriber_;
        ros::Publisher velocity_publisher_;
        SimulatorNode *sim_node_;
        const std::string namespace_;
        const RVO::Vector3 ini_position_;
        RVO::Vector3 targetVelocity_, targetPosition_;
        size_t agentNo_;

        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& data);
        void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& data);

    public:
        Agent(SimulatorNode *node, RVO::Vector3 position, std::string ns="");
        RVO::Vector3 computePrefVelocity() const;
        void sendCmd() const;
        size_t getNumber() const { return agentNo_; }
        void init();
        void setTarget(RVO::Vector3 position, RVO::Vector3 velocity=RVO::Vector3());
    };
}

#endif
