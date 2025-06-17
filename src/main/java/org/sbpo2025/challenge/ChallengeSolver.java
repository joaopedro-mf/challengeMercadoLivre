package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.util.*;
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

    // Estruturas para otimizações
    private List<Integer> validOrders;
    private Map<Integer, Set<Integer>> itemToAisles;
    private Map<Integer, Set<Integer>> orderToRequiredAisles;
    private int realUpperBound;
    private int minAislesNeeded;

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
            obj.setCoefficient(totalAisles, -nItems*1.1); 
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

            System.out.println("Solução não viável ou não ótima encontrada, usando fallback greedy.");
            return greedyFallback();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Erro durante a resolução com SCIP, usando fallback greedy.");
            return greedyFallback();
        }
    }

    private void preprocessData() {
        for (int i = 0; i < nItems; i++) {
            Set<Integer> aislesForItem = new HashSet<>();
            for (int a = 0; a < aisles.size(); a++) {
                if (aisles.get(a).containsKey(i) && aisles.get(a).get(i) > 0) {
                    aislesForItem.add(a);
                }
            }
            itemToAisles.put(i, aislesForItem);
        }

        for (int o = 0; o < orders.size(); o++) {
            Set<Integer> requiredAisles = new HashSet<>();
            Map<Integer, Integer> order = orders.get(o);
            
            for (Map.Entry<Integer, Integer> entry : order.entrySet()) {
                int item = entry.getKey();
                if (itemToAisles.containsKey(item)) {
                    requiredAisles.addAll(itemToAisles.get(item));
                }
            }
            orderToRequiredAisles.put(o, requiredAisles);
        }
        
        validOrders.clear();
        for (int o = 0; o < orders.size(); o++) {
            if (!isOrderDominated(o)) {
                validOrders.add(o);
            }
        }
        
        System.out.println("Ordens válidas após preprocessing: " + validOrders.size() + "/" + orders.size());
    }

    private ChallengeSolution greedyFallback() {
        try {
            Set<Integer> selectedOrders = new HashSet<>();
            Set<Integer> selectedAisles = new HashSet<>();
            int totalItems = 0;
            
            System.out.println("Usando solução greedy de fallback...");
            // Ordenar ordens por eficiência (itens/corredores necessários)
            List<Integer> sortedOrders = new ArrayList<>(validOrders);
            sortedOrders.sort((o1, o2) -> {
                int items1 = orders.get(o1).values().stream().mapToInt(Integer::intValue).sum();
                int items2 = orders.get(o2).values().stream().mapToInt(Integer::intValue).sum();
                int aisles1 = orderToRequiredAisles.get(o1).size();
                int aisles2 = orderToRequiredAisles.get(o2).size();
                
                double eff1 = (double) items1 / Math.max(1, aisles1);
                double eff2 = (double) items2 / Math.max(1, aisles2);
                
                return Double.compare(eff2, eff1); // Ordem decrescente
            });
            
            // Adicionar ordens greedily
            for (int orderIdx : sortedOrders) {
                Map<Integer, Integer> order = orders.get(orderIdx);
                int orderItems = order.values().stream().mapToInt(Integer::intValue).sum();
                
                if (totalItems + orderItems <= realUpperBound) {
                    selectedOrders.add(orderIdx);
                    totalItems += orderItems;
                    
                    // Adicionar corredores necessários
                    selectedAisles.addAll(orderToRequiredAisles.get(orderIdx));
                    
                    if (totalItems >= waveSizeLB) {
                        break;
                    }
                }
            }
            
            if (totalItems >= waveSizeLB && totalItems <= waveSizeUB) {
                return new ChallengeSolution(selectedOrders, selectedAisles);
            }
            
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        return null;
    }
    
    /**
     * Verifica se uma ordem é dominada por outra
     */
    private boolean isOrderDominated(int orderIdx) {
        Map<Integer, Integer> order = orders.get(orderIdx);
        int orderValue = order.values().stream().mapToInt(Integer::intValue).sum();
        Set<Integer> orderAisles = orderToRequiredAisles.get(orderIdx);
        
        for (int otherIdx = 0; otherIdx < orders.size(); otherIdx++) {
            if (otherIdx == orderIdx) continue;
            
            Map<Integer, Integer> otherOrder = orders.get(otherIdx);
            int otherValue = otherOrder.values().stream().mapToInt(Integer::intValue).sum();
            Set<Integer> otherAisles = orderToRequiredAisles.get(otherIdx);
            
            // Outra ordem domina se tem >= valor com <= corredores
            if (otherValue >= orderValue && otherAisles.size() <= orderAisles.size()) {
                // Verificar se todos os itens da ordem atual estão cobertos
                boolean dominated = true;
                for (Map.Entry<Integer, Integer> entry : order.entrySet()) {
                    int item = entry.getKey();
                    int quantity = entry.getValue();
                    
                    Integer otherQuantity = otherOrder.get(item);
                    if (otherQuantity == null || otherQuantity < quantity) {
                        dominated = false;
                        break;
                    }
                }
                
                if (dominated) {
                    return true;
                }
            }
        }
        
        return false;
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
