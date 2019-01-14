/*
 * Sphere.cpp
 * RVO2-3D Library
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

/* Example file showing a demo with 812 agents initially positioned evenly distributed on a sphere attempting to move to the antipodal position on the sphere. */

#define RVOOUTPUTTIMEANDPOSITIONS

using System;
using System.Collections.Generic;
using UnityEngine;

namespace RVO
{
    class Sphere
    {
        /* Store the goals of the agents. */
        IList<Vector3> goals;

        Sphere()
        {
            goals = new List<Vector3>();
        }

        void setupScenario()
        {
            /* Specify the global time step of the simulation. */
            Simulator.Instance.setTimeStep(0.125f);

            /*
             * Specify the default parameters for agents that are subsequently
             * added.
             */
            Simulator.Instance.setAgentDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f, Vector3.zero);

            /*
             * Add agents, specifying their start position, and store their
             * goals on the opposite side of the environment.
             */
            float M_PI = 3.14159265358979323846f;
            /* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */
            for (float a = 0; a < Math.PI; a += 0.1f)
            {
                float z = 100.0f * (float)Math.Cos(a);
                float r = 100.0f * (float)Math.Sin(a);

                for (int i = 0; i < r / 2.5f; ++i)
                {
                    float x = r * (float)Math.Cos(i * 2.0f * M_PI / (r / 2.5f));
                    float y = r * (float)Math.Sin(i * 2.0f * M_PI / (r / 2.5f));

                    Simulator.Instance.addAgent(new Vector3(x, y, z));
                    goals.Add(-Simulator.Instance.getAgentPosition(Simulator.Instance.getNumAgents() - 1));
                }
            }
        }

        #if RVOOUTPUTTIMEANDPOSITIONS
        void updateVisualization()
        {
            /* Output the current global time. */
            Console.Write(Simulator.Instance.getGlobalTime());

            /* Output the current position of all the agents. */
            for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
            {
                Console.Write(" {0}", Simulator.Instance.getAgentPosition(i));
            }

            Console.WriteLine();
        }
        #endif

        void setPreferredVelocities()
        {
            /*
             * Set the preferred velocity to be a vector of unit magnitude
             * (speed) in the direction of the goal.
             */
            for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
            {
                Vector3 goalVector = goals[i] - Simulator.Instance.getAgentPosition(i);

                if (RVOMath.absSq(goalVector) > 1.0f)
                {
                    goalVector = RVOMath.normalize(goalVector);
                }

                Simulator.Instance.setAgentPrefVelocity(i, goalVector);
            }
        }

        bool reachedGoal()
        {
            /* Check if all agents have reached their goals. */
            for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
            {
                if (RVOMath.absSq(Simulator.Instance.getAgentPosition(i) - goals[i]) > Simulator.Instance.getAgentRadius(i) * Simulator.Instance.getAgentRadius(i))
                {
                    return false;
                }
            }
            return true;
        }

        public static void Main(string[] args)
        {
            Sphere sphere = new Sphere();

            /* Set up the scenario. */
            sphere.setupScenario();

            /* Perform (and manipulate) the simulation. */
            do
            {
#if RVOOUTPUTTIMEANDPOSITIONS
                sphere.updateVisualization();
#endif
                sphere.setPreferredVelocities();
                Simulator.Instance.doStep();
            }
            while (!sphere.reachedGoal());
        }
    }
}
