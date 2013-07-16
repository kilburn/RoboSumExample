/*
 * Software License Agreement (BSD License)
 *
 * Copyright 2013 Marc Pujol <mpujol@iiia.csic.es>.
 *
 * Redistribution and use of this software in source and binary forms, with or
 * without modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above
 *   copyright notice, this list of conditions and the
 *   following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other
 *   materials provided with the distribution.
 *
 *   Neither the name of IIIA-CSIC, Artificial Intelligence Research Institute
 *   nor the names of its contributors may be used to
 *   endorse or promote products derived from this
 *   software without specific prior written permission of
 *   IIIA-CSIC, Artificial Intelligence Research Institute
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package es.csic.iiia.rsum;

import es.csic.iiia.maxsum.CommunicationAdapter;
import es.csic.iiia.maxsum.Factor;
import es.csic.iiia.maxsum.MaxOperator;
import es.csic.iiia.maxsum.com.InstantCommunicationAdapter;
import es.csic.iiia.maxsum.factor.CardinalityFactor;
import es.csic.iiia.maxsum.factor.CompositeIndependentFactor;
import es.csic.iiia.maxsum.factor.IndependentFactor;
import es.csic.iiia.maxsum.factor.SelectorFactor;
import es.csic.iiia.maxsum.operator.Minimize;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Example on how to use the binary max-sum library to solve the example
 * problem outlayed in the description pdf.
 * 
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class RoboSumExample {

    /** 
     * Number of iterations to run (100 is way too much for such a small
     * problem, but it doesn't hurt either.
     */
    public static final int ITERATIONS = 100;

    /** K value to use. **/
    public static final double K = 1;
    /** \alpha value to use. **/
    public static final double ALPHA = 2;

    /**
     * We use the instant communication adapter because this is running in
     * a centralized manner.
     */
    private final CommunicationAdapter<Factor> communicationAdapter =
            new InstantCommunicationAdapter<Factor>();

    /** Max operator to use. We chose Minimize because distances are costs **/
    private final MaxOperator maxOperator = new Minimize();

    /** List of all (runnable, non-composited) factors. */
    private List<Factor> factors = new ArrayList<Factor>();

    /**
     * Main method, just to launch the thing.
     * @param args the command line arguments (ignored)
     */
    public static void main(String[] args) {
        new RoboSumExample().launch();
    }

    /**
     * Launches the solver to compute an allocation of tasks to robots.
     */
    public void launch() {
        
        ////////////////////////////////////////////////////////////////////////
        // Problem "definition":

        final int n_tasks = 3;
        final int n_robots = 2;
        String[] task_names = new String[]{"a", "b", "c"};
        
        // Distances from robot r to task [a,b,c]
        double r_to_task[][] = new double[][]{
            {1, 6, 11},     // r1 to a,b,c
            {1, 7, 11},     // r2 to a,b,c
        };


        ////////////////////////////////////////////////////////////////////////
        // Create the factor graph shown in Figure 3 of the document

        // Selector factors
        SelectorFactor<Factor>[] selectors = new SelectorFactor[n_tasks];
        for (int i=0; i<n_tasks; i++) {
            selectors[i] = new SelectorFactor();
            initialize(selectors[i]);
        }
        
        // Combined factor of each robot
        CompositeIndependentFactor[] combinedFactors =
                new CompositeIndependentFactor[n_robots];
        Map<Factor, Integer> factor2robot = new HashMap<Factor, Integer>(n_robots);
        for (int robot = 0; robot<n_robots; robot++) {

            // The independent part
            IndependentFactor ind = new IndependentFactor();
            for (int task = 0; task<n_tasks; task++) {                
                ind.setPotential(selectors[task], r_to_task[robot][task]);
            }

            // The cardinality part
            CardinalityFactor kn = new CardinalityFactor();
            kn.setFunction(new CardinalityFactor.CardinalityFunction() {
                @Override
                public double getCost(int nActiveVariables) {
                    return K * Math.pow(nActiveVariables, ALPHA);
                }
            });

            // Combine them
            combinedFactors[robot] = new CompositeIndependentFactor();
            combinedFactors[robot].setIndependentFactor(ind);
            combinedFactors[robot].setInnerFactor(kn);
            initialize(combinedFactors[robot]);
            factor2robot.put(combinedFactors[robot], robot);
        }

        // Link the composite and the selector factors together
        for (int task=0; task<n_tasks; task++) {
            for (int robot = 0; robot<n_robots; robot++) {
                selectors[task].addNeighbor(combinedFactors[robot]);
                combinedFactors[robot].addNeighbor(selectors[task]);
            }
        }

        
        ////////////////////////////////////////////////////////////////////////
        // Run the algorithm.
        for (int i=0; i<ITERATIONS; i++) {
            for (Factor f : factors) {
                f.run();
            }
        }

        ////////////////////////////////////////////////////////////////////////
        // Exctract allocation
        for (int task=0; task<n_tasks; task++) {
            int robot = factor2robot.get(selectors[task].select());
            System.out.println("Task " + task_names[task] + " => Robot " + (robot+1));
        }

        ////////////////////////////////////////////////////////////////////////
        // Now each robot should compute its path depending on the tasks
        // it has been assigned.
    }

    /**
     * Initializes a factor by setting the communication adapter,
     * max operator, etc.
     *
     * @param f factor to initialize.
     */
    private void initialize(Factor f) {
        f.setIdentity(f);
        f.setCommunicationAdapter(communicationAdapter);
        f.setMaxOperator(maxOperator);
        factors.add(f);
    }

}
