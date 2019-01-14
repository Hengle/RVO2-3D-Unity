/*
 * Agent.cs
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

using System;
using UnityEngine;
using System.Collections.Generic;

namespace RVO
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal class Agent
    {
        internal IList<KeyValuePair<float, Agent>> agentNeighbors = new List<KeyValuePair<float, Agent>>();
        internal IList<KeyValuePair<float, Obstacle>> obstacleNeighbors = new List<KeyValuePair<float, Obstacle>>();
        internal IList<Line> orcaLines = new List<Line>();
        internal Vector3 position;
        internal Vector3 prefVelocity;
        internal Vector3 velocity;
        internal int id = 0;
        internal int maxNeighbors = 0;
        internal float maxSpeed = 0.0f;
        internal float neighborDist = 0.0f;
        internal float radius = 0.0f;
        internal float timeHorizon = 0.0f;
        internal float timeHorizonObst = 0.0f;
        internal bool needDelete = false;

        private Vector3 newVelocity;

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors.Clear();
            float rangeSq = RVOMath.sqr(timeHorizonObst * maxSpeed + radius);
            Simulator.Instance.kdTree.computeObstacleNeighbors(this, rangeSq);

            agentNeighbors.Clear();

            if (maxNeighbors > 0)
            {
                rangeSq = RVOMath.sqr(neighborDist);
                Simulator.Instance.kdTree.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        internal void computeNewVelocity()
        {
            orcaLines.Clear();

            float invTimeHorizonObst = 1.0f / timeHorizonObst;

            /* Create obstacle ORCA lines. */
            for (int i = 0; i < obstacleNeighbors.Count; ++i)
            {

                Obstacle obstacle1 = obstacleNeighbors[i].Value;
                Obstacle obstacle2 = obstacle1.next;

                Vector3 relativePosition1 = obstacle1.point - position;
                Vector3 relativePosition2 = obstacle2.point - position;

                /*
                 * Check if velocity obstacle of obstacle is already taken care
                 * of by previously constructed obstacle ORCA lines.
                 */
                bool alreadyCovered = false;

                for (int j = 0; j < orcaLines.Count; ++j)
                {
                    if (RVOMath.det(invTimeHorizonObst * relativePosition1 - orcaLines[j].point, orcaLines[j].direction) - invTimeHorizonObst * radius >= -RVOMath.RVOEPSILON && RVOMath.det(invTimeHorizonObst * relativePosition2 - orcaLines[j].point, orcaLines[j].direction) - invTimeHorizonObst * radius >= -RVOMath.RVOEPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                /* Not yet covered. Check for collisions. */
                float distSq1 = RVOMath.absSq(relativePosition1);
                float distSq2 = RVOMath.absSq(relativePosition2);

                float radiusSq = RVOMath.sqr(radius);

                Vector3 obstacleVector = obstacle2.point - obstacle1.point;
                float s = Vector3.Dot(-relativePosition1 , obstacleVector) / RVOMath.absSq(obstacleVector);
                float distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

                Line line;

                if (s < 0.0f && distSq1 <= radiusSq)
                {
                    /* Collision with left vertex. Ignore if non-convex. */
                    if (obstacle1.convex)
                    {
                        line.point = new Vector3(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new Vector3(-relativePosition1.y, relativePosition1.x));
                        orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s > 1.0f && distSq2 <= radiusSq)
                {
                    /*
                     * Collision with right vertex. Ignore if non-convex or if
                     * it will be taken care of by neighboring obstacle.
                     */
                    if (obstacle2.convex && RVOMath.det(relativePosition2, obstacle2.direction) >= 0.0f)
                    {
                        line.point = new Vector3(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new Vector3(-relativePosition2.y, relativePosition2.x));
                        orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq)
                {
                    /* Collision with obstacle segment. */
                    line.point = new Vector3(0.0f, 0.0f);
                    line.direction = -obstacle1.direction;
                    orcaLines.Add(line);

                    continue;
                }

                /*
                 * No collision. Compute legs. When obliquely viewed, both legs
                 * can come from a single vertex. Legs extend cut-off line when
                 * non-convex vertex.
                 */

                Vector3 leftLegDirection, rightLegDirection;

                if (s < 0.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that left vertex
                     * defines velocity obstacle.
                     */
                    if (!obstacle1.convex)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle2 = obstacle1;

                    float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                    leftLegDirection = new Vector3(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                    rightLegDirection = new Vector3(relativePosition1.x * leg1 + relativePosition1.y * radius, -relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                }
                else if (s > 1.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that
                     * right vertex defines velocity obstacle.
                     */
                    if (!obstacle2.convex)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle1 = obstacle2;

                    float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                    leftLegDirection = new Vector3(relativePosition2.x * leg2 - relativePosition2.y * radius, relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                    rightLegDirection = new Vector3(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                }
                else
                {
                    /* Usual situation. */
                    if (obstacle1.convex)
                    {
                        float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                        leftLegDirection = new Vector3(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -obstacle1.direction;
                    }

                    if (obstacle2.convex)
                    {
                        float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                        rightLegDirection = new Vector3(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = obstacle1.direction;
                    }
                }

                /*
                 * Legs can never point into neighboring edge when convex
                 * vertex, take cutoff-line of neighboring edge instead. If
                 * velocity projected on "foreign" leg, no constraint is added.
                 */

                Obstacle leftNeighbor = obstacle1.previous;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1.convex && RVOMath.det(leftLegDirection, -leftNeighbor.direction) >= 0.0f)
                {
                    /* Left leg points into obstacle. */
                    leftLegDirection = -leftNeighbor.direction;
                    isLeftLegForeign = true;
                }

                if (obstacle2.convex && RVOMath.det(rightLegDirection, obstacle2.direction) <= 0.0f)
                {
                    /* Right leg points into obstacle. */
                    rightLegDirection = obstacle2.direction;
                    isRightLegForeign = true;
                }

                /* Compute cut-off centers. */
                Vector3 leftCutOff = invTimeHorizonObst * (obstacle1.point - position);
                Vector3 rightCutOff = invTimeHorizonObst * (obstacle2.point - position);
                Vector3 cutOffVector = rightCutOff - leftCutOff;

                /* Project current velocity on velocity obstacle. */

                /* Check if current velocity is projected on cutoff circles. */
                float t = obstacle1 == obstacle2 ? 0.5f : (Vector3.Dot((velocity - leftCutOff) , cutOffVector)) / RVOMath.absSq(cutOffVector);
                float tLeft = Vector3.Dot((velocity - leftCutOff) ,leftLegDirection);
                float tRight = Vector3.Dot((velocity - rightCutOff) , rightLegDirection);

                if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f))
                {
                    /* Project on left cut-off circle. */
                    Vector3 unitW = RVOMath.normalize(velocity - leftCutOff);

                    line.direction = new Vector3(unitW.y, -unitW.x);
                    line.point = leftCutOff + radius * invTimeHorizonObst * unitW;
                    orcaLines.Add(line);

                    continue;
                }
                else if (t > 1.0f && tRight < 0.0f)
                {
                    /* Project on right cut-off circle. */
                    Vector3 unitW = RVOMath.normalize(velocity - rightCutOff);

                    line.direction = new Vector3(unitW.y, -unitW.x);
                    line.point = rightCutOff + radius * invTimeHorizonObst * unitW;
                    orcaLines.Add(line);

                    continue;
                }

                /*
                 * Project on left leg, right leg, or cut-off line, whichever is
                 * closest to velocity.
                 */
                float distSqCutoff = (t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? float.PositiveInfinity : RVOMath.absSq(velocity - (leftCutOff + t * cutOffVector));
                float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : RVOMath.absSq(velocity - (leftCutOff + tLeft * leftLegDirection));
                float distSqRight = tRight < 0.0f ? float.PositiveInfinity : RVOMath.absSq(velocity - (rightCutOff + tRight * rightLegDirection));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* Project on cut-off line. */
                    line.direction = -obstacle1.direction;
                    line.point = leftCutOff + radius * invTimeHorizonObst * new Vector3(-line.direction.y, line.direction.x);
                    orcaLines.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* Project on left leg. */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    line.point = leftCutOff + radius * invTimeHorizonObst * new Vector3(-line.direction.y, line.direction.x);
                    orcaLines.Add(line);

                    continue;
                }

                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutOff + radius * invTimeHorizonObst * new Vector3(-line.direction.y, line.direction.x);
                orcaLines.Add(line);
            }

            int numObstLines = orcaLines.Count;

            float invTimeHorizon = 1.0f / timeHorizon;

            /* Create agent ORCA lines. */
            for (int i = 0; i < agentNeighbors.Count; ++i)
            {
                Agent other = agentNeighbors[i].Value;

                Vector3 relativePosition = other.position - position;
                Vector3 relativeVelocity = velocity - other.velocity;
                float distSq = RVOMath.absSq(relativePosition);
                float combinedRadius = radius + other.radius;
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line;
                Vector3 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    float wLengthSq = RVOMath.absSq(w);
                    float dotProduct1 = Vector3.Dot(w , relativePosition);

                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        float wLength = RVOMath.sqrt(wLengthSq);
                        Vector3 unitW = w / wLength;

                        line.direction = new Vector3(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on legs. */
                        float leg = RVOMath.sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* Project on left leg. */
                            line.direction = new Vector3(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            /* Project on right leg. */
                            line.direction = -new Vector3(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        float dotProduct2 = Vector3.Dot(relativeVelocity ,line.direction);
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    float invTimeStep = 1.0f / Simulator.Instance.timeStep;

                    /* Vector from cutoff center to relative velocity. */
                    Vector3 w = relativeVelocity - invTimeStep * relativePosition;

                    float wLength = RVOMath.abs(w);
                    Vector3 unitW = w / wLength;

                    line.direction = new Vector3(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.point = velocity + 0.5f * u;
                orcaLines.Add(line);
            }

            int lineFail = linearProgram2(orcaLines, maxSpeed, prefVelocity, false, ref newVelocity);

            if (lineFail < orcaLines.Count)
            {
                linearProgram3(orcaLines, numObstLines, lineFail, maxSpeed, ref newVelocity);
            }
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = RVOMath.absSq(position - agent.position);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors.Count < maxNeighbors)
                    {
                        agentNeighbors.Add(new KeyValuePair<float, Agent>(distSq, agent));
                    }

                    int i = agentNeighbors.Count - 1;

                    while (i != 0 && distSq < agentNeighbors[i - 1].Key)
                    {
                        agentNeighbors[i] = agentNeighbors[i - 1];
                        --i;
                    }

                    agentNeighbors[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    if (agentNeighbors.Count == maxNeighbors)
                    {
                        rangeSq = agentNeighbors[agentNeighbors.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, float rangeSq)
        {
            Obstacle nextObstacle = obstacle.next;

            float distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, position);

            if (distSq < rangeSq)
            {
                obstacleNeighbors.Add(new KeyValuePair<float, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors.Count - 1;

                while (i != 0 && distSq < obstacleNeighbors[i - 1].Key)
                {
                    obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                    --i;
                }
                obstacleNeighbors[i] = new KeyValuePair<float, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            velocity = newVelocity;
            position += velocity * Simulator.Instance.timeStep;
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private bool linearProgram1(IList<Line> lines, int lineNo, float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
        {
            float dotProduct = Vector3.Dot(lines[lineNo].point , lines[lineNo].direction);
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVOEPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (Vector3.Dot(optVelocity , lines[lineNo].direction) > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = Vector3.Dot(lines[lineNo].direction ,(optVelocity - lines[lineNo].point));

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private int linearProgram2(IList<Line> lines, float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector3 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector3 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVOEPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (Vector3.Dot(lines[i].direction , lines[j].direction) > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector3 tempResult = result;
                    if (linearProgram2(projLines, radius, new Vector3(-lines[i].direction.y, lines[i].direction.x), true, ref result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }
}
