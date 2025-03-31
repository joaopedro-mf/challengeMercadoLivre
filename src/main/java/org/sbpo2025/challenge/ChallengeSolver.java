package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.concurrent.TimeUnit;

import com.google.ortools.Loader;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        try{        
            // Create solver SCIP 
            Loader.loadNativeLibraries();
            
            MPSolver solver = MPSolver.createSolver("SCIP");
            if (solver == null) {
                System.out.println("Create error SCIP");
                return null;
            }

            int numOrders = orders.size();
            int numAisles = aisles.size();
            
            // Create variables
            MPVariable[] orderVars = new MPVariable[numOrders];
            for (int o = 0; o < numOrders; o++) {
                orderVars[o] = solver.makeBoolVar("order_" + o);
            }
            
            MPVariable[] aisleVars = new MPVariable[numAisles];
            for (int a = 0; a < numAisles; a++) {
                aisleVars[a] = solver.makeBoolVar("aisle_" + a);
            }
            
            MPVariable totalUnits = solver.makeIntVar(0.0, waveSizeUB, "total_units");
            
            MPConstraint totalUnitsConstraint = solver.makeConstraint(0, 0);
            totalUnitsConstraint.setCoefficient(totalUnits, 1);
            for (int o = 0; o < numOrders; o++) {
                int orderTotal = orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
                totalUnitsConstraint.setCoefficient(orderVars[o], -orderTotal);
            }
            
            // lower
            MPConstraint lbConstraint = solver.makeConstraint(waveSizeLB, Double.POSITIVE_INFINITY);
            lbConstraint.setCoefficient(totalUnits, 1);
            
            // upper
            MPConstraint ubConstraint = solver.makeConstraint(0, waveSizeUB);
            ubConstraint.setCoefficient(totalUnits, 1);
            
            // aisles suply to items in orders

            for (int i = 0; i < nItems; i++) {
                MPConstraint itemConstraint = solver.makeConstraint(0, Double.POSITIVE_INFINITY);
                
                for (int a = 0; a < numAisles; a++) {
                    Integer quantity = aisles.get(a).get(i);
                    if (quantity != null && quantity > 0) {
                        itemConstraint.setCoefficient(aisleVars[a], quantity);
                    }
                }
                
                for (int o = 0; o < numOrders; o++) {
                    Integer quantity = orders.get(o).get(i);
                    if (quantity != null && quantity > 0) {
                        itemConstraint.setCoefficient(orderVars[o], -quantity);
                    }
                }
            }
            
            // number of aisles used
            MPVariable totalAisles = solver.makeIntVar(1, numAisles, "total_aisles");
            MPConstraint totalAislesConstraint = solver.makeConstraint(0, 0);
            totalAislesConstraint.setCoefficient(totalAisles, 1);
            for (int a = 0; a < numAisles; a++) {
                totalAislesConstraint.setCoefficient(aisleVars[a], -1);
            }
            
            // Obj: Max itens in aisles
            MPObjective obj = solver.objective();
            obj.setCoefficient(totalUnits, 1);
            obj.setCoefficient(totalAisles, -nItems); 
            obj.setMaximization();
            
            solver.setTimeLimit(getRemainingTime(stopWatch) * 1000);
            
            MPSolver.ResultStatus status = solver.solve();
            
            // solution
            if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
                Set<Integer> selectedOrders = new HashSet<>();
                for (int o = 0; o < numOrders; o++) {
                    if (orderVars[o].solutionValue() > 0.5) {
                        selectedOrders.add(o);
                    }
                }
                
                Set<Integer> selectedAisles = new HashSet<>();
                for (int a = 0; a < numAisles; a++) {
                    if (aisleVars[a].solutionValue() > 0.5) {
                        selectedAisles.add(a);
                    }
                }
                
                ChallengeSolution solution = new ChallengeSolution(selectedOrders, selectedAisles);
                
                if (isSolutionFeasible(solution)) {
                    return solution;
                }
            }

            return null;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /*
     * Get the remaining time in seconds
     */
    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(
                TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
                0);
    }

    protected boolean isSolutionFeasible(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return false;
        }

        int[] totalUnitsPicked = new int[nItems];
        int[] totalUnitsAvailable = new int[nItems];

        // Calculate total units picked
        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnitsPicked[entry.getKey()] += entry.getValue();
            }
        }

        // Calculate total units available
        for (int aisle : visitedAisles) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }

        // Check if the total units picked are within bounds
        int totalUnits = Arrays.stream(totalUnitsPicked).sum();
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
            return false;
        }

        // Check if the units picked do not exceed the units available
        for (int i = 0; i < nItems; i++) {
            if (totalUnitsPicked[i] > totalUnitsAvailable[i]) {
                return false;
            }
        }

        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return 0.0;
        }
        int totalUnitsPicked = 0;

        // Calculate total units picked
        for (int order : selectedOrders) {
            totalUnitsPicked += orders.get(order).values().stream()
                    .mapToInt(Integer::intValue)
                    .sum();
        }

        // Calculate the number of visited aisles
        int numVisitedAisles = visitedAisles.size();

        // Objective function: total units picked / number of visited aisles
        return (double) totalUnitsPicked / numVisitedAisles;
    }
}
