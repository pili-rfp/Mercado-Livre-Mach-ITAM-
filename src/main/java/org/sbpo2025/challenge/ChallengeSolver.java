package org.sbpo2025.challenge;

import ilog.concert.*;
import ilog.cplex.*;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import org.apache.commons.lang3.time.StopWatch;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    public static List<Object> solveForH(IloCplex model, IloRange res, int h, double timeLimit, double gap,
                                 Map<Integer, IloIntVar> x, Map<Integer, IloIntVar> y) throws IloException{
        // Establecer el número de pasillos
      
        res.setBounds(h,h);
                   
        // Parámetros del solver
        model.setParam(IloCplex.Param.TimeLimit, Math.min(120, timeLimit));
        model.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, gap);
        model.setParam(IloCplex.Param.MIP.Strategy.HeuristicFreq,5);
        model.setParam(IloCplex.Param.MIP.Strategy.HeuristicEffort, 1.0);
        model.setParam(IloCplex.Param.MIP.Strategy.RINSHeur, 0);
        model.setParam(IloCplex.Param.MIP.Limits.CutPasses, 5);
        model.setParam(IloCplex.Param.MIP.Cuts.Gomory, 0);
        model.setParam(IloCplex.Param.Preprocessing.NumPass , 2);
        model.setParam(IloCplex.Param.MIP.Display, 4); // Nivel de detalle del log (0 a 5)

        //model.setOut(System.out); // Restaurar la salida a la consola
        
        model.solve();

        // Resolver
        if (model.getStatus() == IloCplex.Status.Infeasible || model.getStatus() == IloCplex.Status.Unknown) {
            return Arrays.asList(-1.0, new HashSet<>(), new HashSet<>());
        }
        else{
            Set<Integer> xVal = new HashSet<>();
            Set<Integer> yVal = new HashSet<>();

            for (Map.Entry<Integer, IloIntVar> entry : x.entrySet()) {
                if (model.getValue(entry.getValue()) > 0.001) {
                    xVal.add(entry.getKey());
                }
            }

            for (Map.Entry<Integer, IloIntVar> entry : y.entrySet()) {
                if (model.getValue(entry.getValue()) > 0.001) {
                    yVal.add(entry.getKey());
                }
            }   

            return Arrays.asList(model.getObjValue(), xVal, yVal);
        }
    }

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

     public ChallengeSolution solve(StopWatch stopWatch) {

        Set<Integer> selectedOrders = new HashSet<>();
        Set<Integer> selectedAisles = new HashSet<>();

        try {
            int nOrders = orders.size();
            int nAisles = aisles.size();
    
            IloCplex cplex = new IloCplex();
            cplex.setName("mlibre");

            cplex.setParam(IloCplex.Param.MIP.Display, 0);
            cplex.setParam(IloCplex.Param.MIP.Cuts.Gomory, -1);
            cplex.setOut(null);

    
            Map<Integer, IloIntVar> x = new HashMap<>();
            Map<Integer, IloIntVar> y = new HashMap<>();
            Map<List<Integer>, IloIntVar> z = new HashMap<>();
    
            // Variables
            for (int i = 0; i < nOrders; i++) {
                x.put(i, cplex.boolVar("x_" + i));
            }
    
            // Variables y y z
            for (int k = 0; k < nAisles; k++) {
                y.put(k, cplex.boolVar("y_" + k));
    
                for (int j = 0; j < nItems; j++) {
                    List<Integer> key = Arrays.asList(k, j);
                    z.put(key, cplex.intVar(0, Integer.MAX_VALUE, "z_" + k + "_" + j));
                }
            }


            // Objetivo: Maximizar cantidad de artículos en wave
            IloLinearNumExpr obj = cplex.linearNumExpr();
            for (int i = 0; i < nOrders; i++) {
                for (Map.Entry<Integer, Integer> product : orders.get(i).entrySet()) {
                    obj.addTerm(product.getValue(), x.get(i));
                }
            }
            cplex.addMaximize(obj);
    
            // Restricciones LB y UB
            IloLinearNumExpr totalExpr = cplex.linearNumExpr();
            for (int i = 0; i < nOrders; i++) {
                for (Map.Entry<Integer, Integer> product : orders.get(i).entrySet()) {
                    totalExpr.addTerm(product.getValue(), x.get(i));
                }
            }
            cplex.addGe(totalExpr, waveSizeLB, "LB_limit");
            cplex.addLe(totalExpr, waveSizeUB, "UB_limit");
    
            // Restricción sobre número de pasillos seleccionados
            IloLinearNumExpr aisleExpr = cplex.linearNumExpr();
            for (int k = 0; k < nAisles; k++) {
                aisleExpr.addTerm(1, y.get(k));
            }
            IloRange restm = cplex.addEq(aisleExpr, 1, "aisles_restriction");
    
            // Restricciones de capacidad
            for (int k = 0; k < nAisles; k++) {
                Map<Integer, Integer> aisleMap = aisles.get(k);
                for (Map.Entry<Integer, Integer> entry : aisleMap.entrySet()) {
                    int j = entry.getKey();
                    int cap = entry.getValue();
                    List<Integer> key = Arrays.asList(k, j);
                    IloLinearNumExpr capExpr = cplex.linearNumExpr();
                    capExpr.addTerm(cap, y.get(k));
                    cplex.addLe(z.get(key), capExpr, "capacity_" + k + "_" + j);
                }
            }
    
            // Restricciones de oferta y demanda
            for (int j = 0; j < nItems; j++) {
                IloLinearNumExpr demandExpr = cplex.linearNumExpr();
                IloLinearNumExpr supplyExpr = cplex.linearNumExpr();
    
                for (int i = 0; i < nOrders; i++) {
                    Map<Integer, Integer> order = orders.get(i);
                    if (order.containsKey(j)) {
                        demandExpr.addTerm(order.get(j), x.get(i));
                    }
                }
    
                for (int k = 0; k < nAisles; k++) {
                    Map<Integer, Integer> aisle = aisles.get(k);
                    if (aisle.containsKey(j)) {
                        List<Integer> key = Arrays.asList(k, j);
                        supplyExpr.addTerm(1.0, z.get(key));
                    }
                }

                cplex.addLe(demandExpr, supplyExpr, "availability_" + j);
            }
    
            // Tiempo disponible y solve inicial
            double remainingTime = getRemainingTime(stopWatch);
            List<Object> s = solveForH(cplex, restm, 1, remainingTime, 0.01, x, y);
    
            double bestRatio = (double) s.get(0) > 0 ? (double) s.get(0) / 1 : -Double.MAX_VALUE;
            int bestK = (double) s.get(0) > 0 ? 1 : nAisles;
            List<Object> bestSol = s;
            int a=1,b=nAisles;
            if ((double) s.get(0) > 0) {
                bestK=1;
                bestSol = s;
                bestRatio = (double) s.get(0);
            }   
            else{
                while (a <= b) {
                    int mid = (a + b) / 2;
                    remainingTime = getRemainingTime(stopWatch);
                    if (mid == 0 || remainingTime < 20.0) break;
                    //System.out.println("Checking for "+ mid + " aisles");
    
                    s = solveForH(cplex, restm, mid, remainingTime - 20.0, 0.07, x, y);
                    //double ratio=0;
                    if ((double) s.get(0) > 0) {
                        double ratio = (double) s.get(0) / mid;
                        if (ratio > bestRatio) {
                            bestRatio = ratio;
                            bestK = mid;
                            bestSol = s;
                        }
                        b = mid - 1;
                    } 
                    else 
                    {
                        a = mid + 1;
                    }
                /*System.out.print(a);
                System.out.print(" ");
                System.out.print(b);
                System.out.print(" ");
                System.out.println(remainingTime);*/

                }
            }

    
            // Imprimir mejor solución
            //System.out.println("Num orders: " + ((Set<Integer>) bestSol.get(1)).size());
            for (int p : (Set<Integer>) bestSol.get(1)) {
                //System.out.println("Order: "+p);
                selectedOrders.add(p);
            }
    
            //System.out.println("Num aisles: " + ((Set<Integer>) bestSol.get(2)).size());
            for (int q : (Set<Integer>) bestSol.get(2)) {
                //System.out.println("Aisle: "+q);
                selectedAisles.add(q);
            }
    
            cplex.end();
           // System.out.println(getRemainingTime(stopWatch));
    
        } catch (IloException e) {
            System.out.println("Error: " + e.getMessage());
        }
    
        
        return new ChallengeSolution(selectedOrders, selectedAisles);
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
