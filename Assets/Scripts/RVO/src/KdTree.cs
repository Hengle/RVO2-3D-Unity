/*
 * KdTree.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

using System.Collections.Generic;
using System;
using UnityEngine;

namespace RVO
{
    /**
     * <summary>Defines k-D trees for agents and static obstacles in the
     * simulation.</summary>
     */
    internal class KdTree
    {
        /**
         * <summary>Defines a node of an agent k-D tree.</summary>
         */
        private struct AgentTreeNode
        {
            internal int begin;
            internal int end;
            internal int left;
            internal int right;
            internal Vector3 minCoord;
            internal Vector3 maxCoord;
        }

        /**
         * <summary>Defines a pair of scalar values.</summary>
         */
        private struct FloatPair
        {
            private float a;
            private float b;

            /**
             * <summary>Constructs and initializes a pair of scalar
             * values.</summary>
             *
             * <param name="a">The first scalar value.</returns>
             * <param name="b">The second scalar value.</returns>
             */
            internal FloatPair(float a_, float b_)
            {
                a = a_;
                b = b_;
            }

            /**
             * <summary>Returns true if the first pair of scalar values is less
             * than the second pair of scalar values.</summary>
             *
             * <returns>True if the first pair of scalar values is less than the
             * second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator <(FloatPair pair1, FloatPair pair2)
            {
                return pair1.a < pair2.a || !(pair2.a < pair1.a) && pair1.b < pair2.b;
            }

            /**
             * <summary>Returns true if the first pair of scalar values is less
             * than or equal to the second pair of scalar values.</summary>
             *
             * <returns>True if the first pair of scalar values is less than or
             * equal to the second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator <=(FloatPair pair1, FloatPair pair2)
            {
                return (pair1.a == pair2.a && pair1.b == pair2.b) || pair1 < pair2;
            }

            /**
             * <summary>Returns true if the first pair of scalar values is
             * greater than the second pair of scalar values.</summary>
             *
             * <returns>True if the first pair of scalar values is greater than
             * the second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator >(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 <= pair2);
            }

            /**
             * <summary>Returns true if the first pair of scalar values is
             * greater than or equal to the second pair of scalar values.
             * </summary>
             *
             * <returns>True if the first pair of scalar values is greater than
             * or equal to the second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator >=(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 < pair2);
            }
        }

        /**
         * <summary>Defines a node of an obstacle k-D tree.</summary>
         */
        private class ObstacleTreeNode
        {
            internal Obstacle obstacle;
            internal ObstacleTreeNode left;
            internal ObstacleTreeNode right;
        };

        /**
         * <summary>The maximum size of an agent k-D tree leaf.</summary>
         */
        private const int MAXLEAFSIZE = 10;

        private Agent[] agents;
        private AgentTreeNode[] agentTree;
        private ObstacleTreeNode obstacleTree;

        /**
         * <summary>Builds an agent k-D tree.</summary>
         */
        internal void buildAgentTree()
        {
            if (agents == null || agents.Length != Simulator.Instance.agents.Count)
            {
                agents = new Agent[Simulator.Instance.agents.Count];

                for (int i = 0; i < agents.Length; ++i)
                {
                    agents[i] = Simulator.Instance.agents[i];
                }

                agentTree = new AgentTreeNode[2 * agents.Length];

                for (int i = 0; i < agentTree.Length; ++i)
                {
                    agentTree[i] = new AgentTreeNode();
                }
            }

            if (agents.Length != 0)
            {
                buildAgentTreeRecursive(0, agents.Length, 0);
            }
        }

        /**
         * <summary>Builds an obstacle k-D tree.</summary>
         */
        internal void buildObstacleTree()
        {
            obstacleTree = new ObstacleTreeNode();

            IList<Obstacle> obstacles = new List<Obstacle>(Simulator.Instance.obstacles.Count);

            for (int i = 0; i < Simulator.Instance.obstacles.Count; ++i)
            {
                obstacles.Add(Simulator.Instance.obstacles[i]);
            }

            obstacleTree = buildObstacleTreeRecursive(obstacles);
        }

        /**
         * <summary>Computes the agent neighbors of the specified agent.
         * </summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
        internal void computeAgentNeighbors(Agent agent, ref float rangeSq)
        {
            queryAgentTreeRecursive(agent, ref rangeSq, 0);
        }

        /**
         * <summary>Computes the obstacle neighbors of the specified agent.
         * </summary>
         *
         * <param name="agent">The agent for which obstacle neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
        internal void computeObstacleNeighbors(Agent agent, float rangeSq)
        {
            queryObstacleTreeRecursive(agent, rangeSq, obstacleTree);
        }

        /**
         * <summary>Queries the visibility between two points within a specified
         * radius.</summary>
         *
         * <returns>True if q1 and q2 are mutually visible within the radius;
         * false otherwise.</returns>
         *
         * <param name="q1">The first point between which visibility is to be
         * tested.</param>
         * <param name="q2">The second point between which visibility is to be
         * tested.</param>
         * <param name="radius">The radius within which visibility is to be
         * tested.</param>
         */
        internal bool queryVisibility(Vector3 q1, Vector3 q2, float radius)
        {
            return queryVisibilityRecursive(q1, q2, radius, obstacleTree);
        }

        internal int queryNearAgent(Vector3 point, float radius)
        {
            float rangeSq = float.MaxValue;
            int agentNo = -1;
            queryAgentTreeRecursive(point, ref rangeSq, ref agentNo, 0);
            if (rangeSq < radius*radius)
                return agentNo;
            return -1;
        }

        /**
         * <summary>Recursive method for building an agent k-D tree.</summary>
         *
         * <param name="begin">The beginning agent k-D tree node node index.
         * </param>
         * <param name="end">The ending agent k-D tree node index.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void buildAgentTreeRecursive(int begin, int end, int node)
        {
            agentTree[node].begin = begin;
            agentTree[node].end = end;
            agentTree[node].minCoord = agents[begin].position;
            agentTree[node].maxCoord = agents[begin].position;

            for (int i = begin + 1; i < end; ++i)
            {
                agentTree[node].maxCoord[0] = Math.Max(agentTree[node].maxCoord[0], agents[i].position.x);
                agentTree[node].minCoord[0] = Math.Min(agentTree[node].minCoord[0], agents[i].position.x);
                agentTree[node].maxCoord[1] = Math.Max(agentTree[node].maxCoord[1], agents[i].position.y);
                agentTree[node].minCoord[1] = Math.Min(agentTree[node].minCoord[1], agents[i].position.y);
                agentTree[node].maxCoord[2] = Math.Max(agentTree[node].maxCoord[2], agents[i].position.z);
                agentTree[node].minCoord[2] = Math.Min(agentTree[node].minCoord[2], agents[i].position.z);
            }

            if (end - begin > MAXLEAFSIZE)
            {
                int coord = 0;
                if (agentTree[node].maxCoord[0] - agentTree[node].minCoord[0] > agentTree[node].maxCoord[1] - agentTree[node].minCoord[1] && agentTree[node].maxCoord[0] - agentTree[node].minCoord[0] > agentTree[node].maxCoord[2] - agentTree[node].minCoord[2])
                {
                    coord = 0;
                }
                else if (agentTree[node].maxCoord[1] - agentTree[node].minCoord[1] > agentTree[node].maxCoord[2] - agentTree[node].minCoord[2])
                {
                    coord = 1;
                }
                else
                {
                    coord = 2;
                }

                /* No leaf node. */
               
                float splitValue = 0.5f * (agentTree[node].maxCoord[coord] + agentTree[node].minCoord[coord]);
                int left = begin;
                int right = end;
                while (left < right)
                {
                    while (left < right && agents[left].position[coord] < splitValue)
                    {
                        ++left;
                    }

                    while (right > left && agents[right - 1].position[coord] >= splitValue)
                    {
                        --right;
                    }

                    if (left < right)
                    {
                        Agent tempAgent = agents[left];
                        agents[left] = agents[right - 1];
                        agents[right - 1] = tempAgent;
                        ++left;
                        --right;
                    }
                }

                int leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                    ++right;
                }

                agentTree[node].left = node + 1;
                agentTree[node].right = node + 2 * leftSize;

                buildAgentTreeRecursive(begin, left, agentTree[node].left);
                buildAgentTreeRecursive(left, end, agentTree[node].right);
            }
        }

        /**
         * <summary>Recursive method for building an obstacle k-D tree.
         * </summary>
         *
         * <returns>An obstacle k-D tree node.</returns>
         *
         * <param name="obstacles">A list of obstacles.</param>
         */
        private ObstacleTreeNode buildObstacleTreeRecursive(IList<Obstacle> obstacles)
        {
            if (obstacles.Count == 0)
            {
                return null;
            }

            ObstacleTreeNode node = new ObstacleTreeNode();

            int optimalSplit = 0;
            int minLeft = obstacles.Count;
            int minRight = obstacles.Count;

            for (int i = 0; i < obstacles.Count; ++i)
            {
                int leftSize = 0;
                int rightSize = 0;

                Obstacle obstacleI1 = obstacles[i];
                Obstacle obstacleI2 = obstacleI1.next;

                /* Compute optimal split node. */
                for (int j = 0; j < obstacles.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    Obstacle obstacleJ1 = obstacles[j];
                    Obstacle obstacleJ2 = obstacleJ1.next;

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                    if (j1LeftOfI >= -RVOMath.RVOEPSILON && j2LeftOfI >= -RVOMath.RVOEPSILON)
                    {
                        ++leftSize;
                    }
                    else if (j1LeftOfI <= RVOMath.RVOEPSILON && j2LeftOfI <= RVOMath.RVOEPSILON)
                    {
                        ++rightSize;
                    }
                    else
                    {
                        ++leftSize;
                        ++rightSize;
                    }

                    if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) >= new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                    {
                        break;
                    }
                }

                if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) < new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* Build split node. */
                IList<Obstacle> leftObstacles = new List<Obstacle>(minLeft);

                for (int n = 0; n < minLeft; ++n)
                {
                    leftObstacles.Add(null);
                }

                IList<Obstacle> rightObstacles = new List<Obstacle>(minRight);

                for (int n = 0; n < minRight; ++n)
                {
                    rightObstacles.Add(null);
                }

                int leftCounter = 0;
                int rightCounter = 0;
                int i = optimalSplit;

                Obstacle obstacleI1 = obstacles[i];
                Obstacle obstacleI2 = obstacleI1.next;

                for (int j = 0; j < obstacles.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    Obstacle obstacleJ1 = obstacles[j];
                    Obstacle obstacleJ2 = obstacleJ1.next;

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                    if (j1LeftOfI >= -RVOMath.RVOEPSILON && j2LeftOfI >= -RVOMath.RVOEPSILON)
                    {
                        leftObstacles[leftCounter++] = obstacles[j];
                    }
                    else if (j1LeftOfI <= RVOMath.RVOEPSILON && j2LeftOfI <= RVOMath.RVOEPSILON)
                    {
                        rightObstacles[rightCounter++] = obstacles[j];
                    }
                    else
                    {
                        /* Split obstacle j. */
                        float t = RVOMath.det(obstacleI2.point - obstacleI1.point, obstacleJ1.point - obstacleI1.point) / RVOMath.det(obstacleI2.point - obstacleI1.point, obstacleJ1.point - obstacleJ2.point);

                        Vector3 splitPoint = obstacleJ1.point + t * (obstacleJ2.point - obstacleJ1.point);

                        Obstacle newObstacle = new Obstacle();
                        newObstacle.point = splitPoint;
                        newObstacle.previous = obstacleJ1;
                        newObstacle.next = obstacleJ2;
                        newObstacle.convex = true;
                        newObstacle.direction = obstacleJ1.direction;

                        newObstacle.id = Simulator.Instance.obstacles.Count;

                        Simulator.Instance.obstacles.Add(newObstacle);

                        obstacleJ1.next = newObstacle;
                        obstacleJ2.previous = newObstacle;

                        if (j1LeftOfI > 0.0f)
                        {
                            leftObstacles[leftCounter++] = obstacleJ1;
                            rightObstacles[rightCounter++] = newObstacle;
                        }
                        else
                        {
                            rightObstacles[rightCounter++] = obstacleJ1;
                            leftObstacles[leftCounter++] = newObstacle;
                        }
                    }
                }

                node.obstacle = obstacleI1;
                node.left = buildObstacleTreeRecursive(leftObstacles);
                node.right = buildObstacleTreeRecursive(rightObstacles);

                return node;
            }
        }

        private void queryAgentTreeRecursive(Vector3 position, ref float rangeSq, ref int agentNo, int node)
        {
            if (agentTree[node].end - agentTree[node].begin <= MAXLEAFSIZE)
            {
                for (int i = agentTree[node].begin; i < agentTree[node].end; ++i)
                {
                    float distSq = RVOMath.absSq(position - agents[i].position);
                    if (distSq < rangeSq)
                    {
                        rangeSq = distSq;
                        agentNo = agents[i].id;
                    }
                }
            }
            else
            {
                float distSqLeft = RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left].minCoord[0] - position.x)) + 
                    RVOMath.sqr(Math.Max(0.0f, position.x - agentTree[agentTree[node].left].maxCoord[0])) + 
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left].minCoord[1] - position.y)) + 
                    RVOMath.sqr(Math.Max(0.0f, position.y - agentTree[agentTree[node].left].maxCoord[1])) +
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left].minCoord[2] - position.z)) +
                    RVOMath.sqr(Math.Max(0.0f, position.z - agentTree[agentTree[node].left].maxCoord[2]));
                float distSqRight = RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right].minCoord[0] - position.x)) + 
                    RVOMath.sqr(Math.Max(0.0f, position.x - agentTree[agentTree[node].right].maxCoord[0])) + 
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right].minCoord[1] - position.y)) +
                    RVOMath.sqr(Math.Max(0.0f, position.y - agentTree[agentTree[node].right].maxCoord[1])) + 
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right].minCoord[2] - position.z)) +
                    RVOMath.sqr(Math.Max(0.0f, position.z - agentTree[agentTree[node].right].maxCoord[2]));

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree[node].left);

                        if (distSqRight < rangeSq)
                        {
                            queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree[node].right);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree[node].right);

                        if (distSqLeft < rangeSq)
                        {
                            queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree[node].left);
                        }
                    }
                }

            }
        }

        /**
         * <summary>Recursive method for computing the agent neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void queryAgentTreeRecursive(Agent agent, ref float rangeSq, int node)
        {
            if (agentTree[node].end - agentTree[node].begin <= MAXLEAFSIZE)
            {
                for (int i = agentTree[node].begin; i < agentTree[node].end; ++i)
                {
                    agent.insertAgentNeighbor(agents[i], ref rangeSq);
                }
            }
            else
            {
                float distSqLeft = RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left].minCoord[0] - agent.position.x)) + 
                    RVOMath.sqr(Math.Max(0.0f, agent.position.x - agentTree[agentTree[node].left].maxCoord[0])) + 
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left].minCoord[1] - agent.position.y)) + 
                    RVOMath.sqr(Math.Max(0.0f, agent.position.y - agentTree[agentTree[node].left].maxCoord[1]))+ 
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left].minCoord[2] - agent.position.z)) + 
                    RVOMath.sqr(Math.Max(0.0f, agent.position.z - agentTree[agentTree[node].left].maxCoord[2]));
                float distSqRight = RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right].minCoord[0] - agent.position.x)) + 
                    RVOMath.sqr(Math.Max(0.0f, agent.position.x - agentTree[agentTree[node].right].maxCoord[0])) + 
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right].minCoord[1] - agent.position.y)) +
                    RVOMath.sqr(Math.Max(0.0f, agent.position.y - agentTree[agentTree[node].right].maxCoord[1])) +
                    RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right].minCoord[2] - agent.position.z)) +
                    RVOMath.sqr(Math.Max(0.0f, agent.position.z - agentTree[agentTree[node].right].maxCoord[2]));

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        queryAgentTreeRecursive(agent, ref rangeSq, agentTree[node].left);

                        if (distSqRight < rangeSq)
                        {
                            queryAgentTreeRecursive(agent, ref rangeSq, agentTree[node].right);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        queryAgentTreeRecursive(agent, ref rangeSq, agentTree[node].right);

                        if (distSqLeft < rangeSq)
                        {
                            queryAgentTreeRecursive(agent, ref rangeSq, agentTree[node].left);
                        }
                    }
                }

            }
        }

        /**
         * <summary>Recursive method for computing the obstacle neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which obstacle neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current obstacle k-D node.</param>
         */
        private void queryObstacleTreeRecursive(Agent agent, float rangeSq, ObstacleTreeNode node)
        {
            if (node != null)
            {
                Obstacle obstacle1 = node.obstacle;
                Obstacle obstacle2 = obstacle1.next;

                float agentLeftOfLine = RVOMath.leftOf(obstacle1.point, obstacle2.point, agent.position);

                queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node.left : node.right);

                float distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(obstacle2.point - obstacle1.point);

                if (distSqLine < rangeSq)
                {
                    if (agentLeftOfLine < 0.0f)
                    {
                        /*
                         * Try obstacle at this node only if agent is on right side of
                         * obstacle (and can see obstacle).
                         */
                        agent.insertObstacleNeighbor(node.obstacle, rangeSq);
                    }

                    /* Try other side of line. */
                    queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node.right : node.left);
                }
            }
        }

        /**
         * <summary>Recursive method for querying the visibility between two
         * points within a specified radius.</summary>
         *
         * <returns>True if q1 and q2 are mutually visible within the radius;
         * false otherwise.</returns>
         *
         * <param name="q1">The first point between which visibility is to be
         * tested.</param>
         * <param name="q2">The second point between which visibility is to be
         * tested.</param>
         * <param name="radius">The radius within which visibility is to be
         * tested.</param>
         * <param name="node">The current obstacle k-D node.</param>
         */
        private bool queryVisibilityRecursive(Vector3 q1, Vector3 q2, float radius, ObstacleTreeNode node)
        {
            if (node == null)
            {
                return true;
            }

            Obstacle obstacle1 = node.obstacle;
            Obstacle obstacle2 = obstacle1.next;

            float q1LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q1);
            float q2LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q2);
            float invLengthI = 1.0f / RVOMath.absSq(obstacle2.point - obstacle1.point);

            if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
            {
                return queryVisibilityRecursive(q1, q2, radius, node.left) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.right));
            }

            if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f)
            {
                return queryVisibilityRecursive(q1, q2, radius, node.right) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.left));
            }

            if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
            {
                /* One can see through obstacle from left to right. */
                return queryVisibilityRecursive(q1, q2, radius, node.left) && queryVisibilityRecursive(q1, q2, radius, node.right);
            }

            float point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point);
            float point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point);
            float invLengthQ = 1.0f / RVOMath.absSq(q2 - q1);

            return point1LeftOfQ * point2LeftOfQ >= 0.0f && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node.left) && queryVisibilityRecursive(q1, q2, radius, node.right);
        }
    }
}
