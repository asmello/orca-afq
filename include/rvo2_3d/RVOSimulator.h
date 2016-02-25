/*
 * RVOSimulator.h
 * RVO2-3D Library
 *
 * Copyright (c) 2008-2013 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <geom@cs.unc.edu> or the Office of
 * Technology Development at the University of North Carolina at Chapel Hill
 * <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/**
 * \file    RVOSimulator.h
 * \brief   Contains the RVOSimulator class.
 */
#ifndef RVO_RVO_SIMULATOR_H_
#define RVO_RVO_SIMULATOR_H_

#include "API.h"

#include <cstddef>
#include <limits>
#include <vector>

#include "Vector3.h"

namespace RVO {
	class Agent;
	class KdTree;

	/**
	 * \brief   Error value.
	 *
	 * A value equal to the largest unsigned integer, which is returned in case of an error by functions in RVO::RVOSimulator.
	 */
	const size_t RVO_ERROR = std::numeric_limits<size_t>::max();

	/**
	 * \brief   Defines a plane.
	 */
	class Plane {
	public:
		/**
		 * \brief   A point on the plane.
		 */
		Vector3 point;

		/**
		 * \brief   The normal to the plane.
		 */
		Vector3 normal;
	};

	/**
	 * \brief  Defines the simulation.
	 *
	 * The main class of the library that contains all simulation functionality.
	 */
	class RVOSimulator {
	public:
		/**
		 * \brief   Constructs a simulator instance.
		 */
		RVO_API RVOSimulator();

		/**
		 * \brief   Constructs a simulator instance and sets the default properties for any new agent that is added.
		 * \param   timeStep      The time step of the simulation. Must be positive.
		 * \param   neighborDist  The default maximum distance (center point to center point) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
		 * \param   maxNeighbors  The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
		 * \param   timeHorizon   The default minimum amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
		 * \param   radius        The default radius of a new agent. Must be non-negative.
		 * \param   maxSpeed      The default maximum speed of a new agent. Must be non-negative.
		 * \param   velocity      The default initial three-dimensional linear velocity of a new agent (optional).
		 */
		RVO_API RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity = Vector3());

		/**
		 * \brief   Destroys this simulator instance.
		 */
		RVO_API ~RVOSimulator();

		/**
		 * \brief   Adds a new agent with default properties to the simulation.
		 * \param   position  The three-dimensional starting position of this agent.
		 * \return  The number of the agent, or RVO::RVO_ERROR when the agent defaults have not been set.
		 */
		RVO_API size_t addAgent(const Vector3 &position);

		/**
		 * \brief   Adds a new agent to the simulation.
		 * \param   position      The three-dimensional starting position of this agent.
		 * \param   neighborDist  The maximum distance (center point to center point) to other agents this agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
		 * \param   maxNeighbors  The maximum number of other agents this agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
		 * \param   timeHorizon   The minimum amount of time for which this agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner this agent will respond to the presence of other agents, but the less freedom this agent has in choosing its velocities. Must be positive.
		 * \param   radius        The radius of this agent. Must be non-negative.
		 * \param   maxSpeed      The maximum speed of this agent. Must be non-negative.
		 * \param   velocity      The initial three-dimensional linear velocity of this agent (optional).
		 * \return  The number of the agent.
		 */
		RVO_API size_t addAgent(const Vector3 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity = Vector3());

		/**
		 * \brief   Lets the simulator perform a simulation step and updates the three-dimensional position and three-dimensional velocity of each agent.
		 */
		RVO_API void doStep();

		/**
		 * \brief   Returns the specified agent neighbor of the specified agent.
		 * \param   agentNo     The number of the agent whose agent neighbor is to be retrieved.
		 * \param   neighborNo  The number of the agent neighbor to be retrieved.
		 * \return  The number of the neighboring agent.
		 */
		RVO_API size_t getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const;

		/**
		 * \brief   Returns the maximum neighbor count of a specified agent.
		 * \param   agentNo  The number of the agent whose maximum neighbor count is to be retrieved.
		 * \return  The present maximum neighbor count of the agent.
		 */
		RVO_API size_t getAgentMaxNeighbors(size_t agentNo) const;

		/**
		 * \brief   Returns the maximum speed of a specified agent.
		 * \param   agentNo  The number of the agent whose maximum speed is to be retrieved.
		 * \return  The present maximum speed of the agent.
		 */
		RVO_API float getAgentMaxSpeed(size_t agentNo) const;

		/**
		 * \brief   Returns the maximum neighbor distance of a specified agent.
		 * \param   agentNo  The number of the agent whose maximum neighbor distance is to be retrieved.
		 * \return  The present maximum neighbor distance of the agent.
		 */
		RVO_API float getAgentNeighborDist(size_t agentNo) const;

		/**
		 * \brief   Returns the count of agent neighbors taken into account to compute the current velocity for the specified agent.
		 * \param   agentNo  The number of the agent whose count of agent neighbors is to be retrieved.
		 * \return  The count of agent neighbors taken into account to compute the current velocity for the specified agent.
		 */
		RVO_API size_t getAgentNumAgentNeighbors(size_t agentNo) const;

		/**
		 * \brief   Returns the count of ORCA constraints used to compute the current velocity for the specified agent.
		 * \param   agentNo  The number of the agent whose count of ORCA constraints is to be retrieved.
		 * \return  The count of ORCA constraints used to compute the current velocity for the specified agent.
		 */
		RVO_API size_t getAgentNumORCAPlanes(size_t agentNo) const;

		/**
		 * \brief   Returns the specified ORCA constraint of the specified agent.
		 * \param   agentNo  The number of the agent whose ORCA constraint is to be retrieved.
		 * \param   planeNo  The number of the ORCA constraint to be retrieved.
		 * \return  A plane representing the specified ORCA constraint.
		 * \note    The halfspace to which the normal of the plane points is the region of permissible velocities with respect to the specified ORCA constraint.
		 */
		RVO_API const Plane &getAgentORCAPlane(size_t agentNo, size_t planeNo) const;

		/**
		 * \brief   Returns the three-dimensional position of a specified agent.
		 * \param   agentNo  The number of the agent whose three-dimensional position is to be retrieved.
		 * \return  The present three-dimensional position of the (center of the) agent.
		 */
		RVO_API const Vector3 &getAgentPosition(size_t agentNo) const;

		/**
		 * \brief   Returns the three-dimensional preferred velocity of a specified agent.
		 * \param   agentNo  The number of the agent whose three-dimensional preferred velocity is to be retrieved.
		 * \return  The present three-dimensional preferred velocity of the agent.
		 */
		RVO_API const Vector3 &getAgentPrefVelocity(size_t agentNo) const;

		/**
		 * \brief   Returns the radius of a specified agent.
		 * \param   agentNo  The number of the agent whose radius is to be retrieved.
		 * \return  The present radius of the agent.
		 */
		RVO_API float getAgentRadius(size_t agentNo) const;

		/**
		 * \brief   Returns the time horizon of a specified agent.
		 * \param   agentNo  The number of the agent whose time horizon is to be retrieved.
		 * \return  The present time horizon of the agent.
		 */
		RVO_API float getAgentTimeHorizon(size_t agentNo) const;

		/**
		 * \brief   Returns the three-dimensional linear velocity of a specified agent.
		 * \param   agentNo  The number of the agent whose three-dimensional linear velocity is to be retrieved.
		 * \return  The present three-dimensional linear velocity of the agent.
		 */
		RVO_API const Vector3 &getAgentVelocity(size_t agentNo) const;

		/**
		 * \brief   Returns the global time of the simulation.
		 * \return  The present global time of the simulation (zero initially).
		 */
		RVO_API float getGlobalTime() const;

		/**
		 * \brief   Returns the count of agents in the simulation.
		 * \return  The count of agents in the simulation.
		 */
		RVO_API size_t getNumAgents() const;

		/**
		 * \brief   Returns the time step of the simulation.
		 * \return  The present time step of the simulation.
		 */
		RVO_API float getTimeStep() const;

		/**
		 * \brief   Removes an agent from the simulation.
		 * \param   agentNo  The number of the agent that is to be removed.
		 * \note    After the removal of the agent, the agent that previously had number getNumAgents() - 1 will now have number agentNo.
		 */
		RVO_API void removeAgent(size_t agentNo);

		/**
		 * \brief   Sets the default properties for any new agent that is added.
		 * \param   neighborDist  The default maximum distance (center point to center point) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
		 * \param   maxNeighbors  The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
		 * \param   timeHorizon   The default minimum amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
		 * \param   radius        The default radius of a new agent. Must be non-negative.
		 * \param   maxSpeed      The default maximum speed of a new agent. Must be non-negative.
		 * \param   velocity      The default initial three-dimensional linear velocity of a new agent (optional).
		 */
		RVO_API void setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity = Vector3());

		/**
		 * \brief   Sets the maximum neighbor count of a specified agent.
		 * \param   agentNo       The number of the agent whose maximum neighbor count is to be modified.
		 * \param   maxNeighbors  The replacement maximum neighbor count.
		 */
		RVO_API void setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors);

		/**
		 * \brief   Sets the maximum speed of a specified agent.
		 * \param   agentNo   The number of the agent whose maximum speed is to be modified.
		 * \param   maxSpeed  The replacement maximum speed. Must be non-negative.
		 */
		RVO_API void setAgentMaxSpeed(size_t agentNo, float maxSpeed);

		/**
		 * \brief   Sets the maximum neighbor distance of a specified agent.
		 * \param   agentNo       The number of the agent whose maximum neighbor distance is to be modified.
		 * \param   neighborDist  The replacement maximum neighbor distance. Must be non-negative.
		 */
		RVO_API void setAgentNeighborDist(size_t agentNo, float neighborDist);

		/**
		 * \brief   Sets the three-dimensional position of a specified agent.
		 * \param   agentNo   The number of the agent whose three-dimensional position is to be modified.
		 * \param   position  The replacement of the three-dimensional position.
		 */
		RVO_API void setAgentPosition(size_t agentNo, const Vector3 &position);

		/**
		 * \brief   Sets the three-dimensional preferred velocity of a specified agent.
		 * \param   agentNo       The number of the agent whose three-dimensional preferred velocity is to be modified.
		 * \param   prefVelocity  The replacement of the three-dimensional preferred velocity.
		 */
		RVO_API void setAgentPrefVelocity(size_t agentNo, const Vector3 &prefVelocity);

		/**
		 * \brief   Sets the radius of a specified agent.
		 * \param   agentNo  The number of the agent whose radius is to be modified.
		 * \param   radius   The replacement radius. Must be non-negative.
		 */
		RVO_API void setAgentRadius(size_t agentNo, float radius);

		/**
		 * \brief   Sets the time horizon of a specified agent with respect to other agents.
		 * \param   agentNo      The number of the agent whose time horizon is to be modified.
		 * \param   timeHorizon  The replacement time horizon with respect to other agents. Must be positive.
		 */
		RVO_API void setAgentTimeHorizon(size_t agentNo, float timeHorizon);

		/**
		 * \brief   Sets the three-dimensional linear velocity of a specified agent.
		 * \param   agentNo   The number of the agent whose three-dimensional linear velocity is to be modified.
		 * \param   velocity  The replacement three-dimensional linear velocity.
		 */
		RVO_API void setAgentVelocity(size_t agentNo, const Vector3 &velocity);

		/**
		 * \brief   Sets the time step of the simulation.
		 * \param   timeStep  The time step of the simulation. Must be positive.
		 */
		RVO_API void setTimeStep(float timeStep);
		
	private:
		Agent *defaultAgent_;
		KdTree *kdTree_;
		float globalTime_;
		float timeStep_;
		std::vector<Agent *> agents_;
		
		friend class Agent;
		friend class KdTree;
	};
}

#endif
