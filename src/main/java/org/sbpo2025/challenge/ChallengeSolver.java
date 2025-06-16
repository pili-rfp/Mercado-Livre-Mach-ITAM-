package org.sbpo2025.challenge;

import ilog.concert.*;
import ilog.cplex.*;
import java.util.*;
import java.util.concurrent.TimeUnit;
import org.apache.commons.lang3.time.StopWatch;


public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes
    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;
   
    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),0);
    }
   /*public static List<Object> solveForH(IloCplex model, IloRange res, int h, double timeLimit, double gap,
    Map<Integer, IloIntVar> x, Map<Integer, IloIntVar> y) throws IloException{*/

   public static List<Object> solveForH(IloCplex model, IloRange resL, IloRange resU, int lb, int ub, double timeLimit, double gap,
                                 Map<Integer, IloIntVar> x, Map<Integer, IloIntVar> y) throws IloException{
        int yTotal = 0;


        // Si no hay suficiente tiempo para ejecutar, se cancela
        if (timeLimit < 20){
            return Arrays.asList(-2.0, new HashSet<>(), new HashSet<>(), -2);
        }

        // Establecer cota superior sobre el número de pasillos
        resL.setLB(lb);
        resU.setUB(ub);

        // Parámetros del solver
        model.setParam(IloCplex.Param.TimeLimit, Math.min(100, timeLimit));
        model.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, gap);

        //model.exportModel("modelowave.lp"); 

        // Resolver
        model.solve();
        if (model.getStatus() == IloCplex.Status.Infeasible || model.getStatus() == IloCplex.Status.Unknown) {
            return Arrays.asList(-1.0, new HashSet<>(), new HashSet<>(), -1);
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
                    yTotal++;
                }
            }   


            return Arrays.asList(model.getObjValue(), xVal, yVal, yTotal);
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
            cplex.setOut(null);
    
            Map<Integer, IloIntVar> x = new HashMap<>();
            Map<Integer, IloIntVar> y = new HashMap<>();
    
            // Variables x (ordenes)
            for (int i = 0; i < nOrders; i++) {
                x.put(i, cplex.boolVar("x_" + i));
            }

            // Variables y (pasillos)
            for (int k = 0; k < nAisles; k++) {
                y.put(k, cplex.boolVar("y_" + k));
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
    
            // Restricciones sobre número de pasillos seleccionados
            IloLinearNumExpr aisleExpr = cplex.linearNumExpr();
            for (int k = 0; k < nAisles; k++) {
                aisleExpr.addTerm(1, y.get(k));
            }
            IloRange restlb = cplex.addGe(aisleExpr, 1, "lb_aisles_restriction");
            IloRange restub = cplex.addLe(aisleExpr, nAisles, "ub_aisles_restriction");
    
            // Crear un diccionario de productos a órdenes para acelerar las restricciones
            Map<Integer, List<List<Integer>>> productToOrders = new HashMap<>();
            for (int j = 0; j < nItems; j++) {
                productToOrders.put(j, new ArrayList<>());   
            }
            // Recorrer las órdenes para llenar el diccionario
            for (int i = 0; i < nOrders; i++) {
                Map<Integer, Integer> order = orders.get(i);
                for (Map.Entry<Integer, Integer> entry : order.entrySet()) {
                    int productIndex = entry.getKey();      // Producto
                    int quantity = entry.getValue();        // Cantidad

                    // Crear lista [i, quantity] y agregarla al producto correspondiente
                    List<Integer> entryPair = Arrays.asList(i, quantity);
                    productToOrders.get(productIndex).add(entryPair);
                }
            }
    
            // Restricciones de oferta y demanda
            for (int j = 0; j < nItems; j++) {
                IloLinearNumExpr demandExpr = cplex.linearNumExpr();
                IloLinearNumExpr supplyExpr = cplex.linearNumExpr();
    
                 // Construir expresión de demanda usando productToOrders
                for (List<Integer> pair : productToOrders.get(j)) {
                    int i = pair.get(0);       // Índice de orden
                    int quantity = pair.get(1); // Cantidad del producto j en la orden i
                    demandExpr.addTerm(quantity, x.get(i));
                }
                
                // Construir expresión de oferta
                for (int k = 0; k < nAisles; k++) {
                    Map<Integer, Integer> aisle = aisles.get(k);
                    if (aisle.containsKey(j)) {
                        supplyExpr.addTerm(aisle.get(j), y.get(k));
                    }
                }
                cplex.addLe(demandExpr, supplyExpr, "availability_" + j);
            }

            // Parametros del solver
            
            //cplex.setParam(IloCplex.Param.NodeAlgorithm,4 );
            cplex.setParam(IloCplex.Param.MIP.Strategy.Branch, 1);
            cplex.setParam(IloCplex.Param.Parallel,-1);
            cplex.setParam(IloCplex.Param.MIP.Strategy.HeuristicFreq,20);
            //cplex.setParam(IloCplex.Param.RootAlgorithm,4);
            cplex.setParam(IloCplex.Param.MIP.Strategy.VariableSelect,1);
            cplex.setParam(IloCplex.Param.MIP.Cuts.MIRCut,2);
            cplex.setParam(IloCplex.Param.MIP.Limits.CutPasses,3);
            cplex.setParam(IloCplex.Param.Preprocessing.Dual,1);
            cplex.setParam(IloCplex.Param.MIP.Display, 4); // Nivel de detalle del log (0 a 5)
            cplex.exportModel("modelowave.lp");
            //java.io.PrintStream out = System.out;
            //cplex.setOut(out);
            // Busqueda binaria
            int a=1,b=nAisles;
            double bestRatio =  -Double.MAX_VALUE;
            double remainingTime;
            int bestK = nAisles;
            List<Object> bestSol = Arrays.asList(-2, new HashSet<>(), new HashSet<>(), -2);
            int i=1,mid;
            while (a < b) {
                if(i==1){
                    int divisiones=(int) Math.floor(Math.log(a+b)/Math.log(2));                
                    mid = (int) Math.floor((a + b) / divisiones);
                    i++;
                }
                else{
                    mid = (int) Math.floor((a + b) / 2.0);
                    i++; 
                }
                
                // Resolver parte izquierda
                remainingTime = getRemainingTime(stopWatch);
                //System.out.print("Tiempo: ");
                //System.out.println(remainingTime);
                List<Object> sIzq = solveForH(cplex, restlb, restub, a, mid, remainingTime - 20, 0.02, x, y);
                System.out.println("izq " + a + " " + mid + " " + sIzq.get(0) + " " + sIzq.get(3) + " " + ((double) sIzq.get(0) / (int) sIzq.get(3)) + " " + remainingTime);

                // Resolver parte derecha
                remainingTime = getRemainingTime(stopWatch);
                List<Object> sDer = solveForH(cplex, restlb, restub, mid + 1, b, remainingTime - 20, 0.02, x, y);
                System.out.println("der " + (mid + 1) + " " + b + " " + sDer.get(0) + " " + sDer.get(3) + " " + ((double) sDer.get(0) / (int) sDer.get(3)) + " " + remainingTime);
                if ((double) sIzq.get(0) / (int) sIzq.get(3) > (double) sDer.get(0) / (int) sDer.get(3)) {
                    double r = (double) sIzq.get(0) / (int) sIzq.get(3);
                    b = mid;
                    if (r > bestRatio) {
                        bestRatio = r;
                        bestK = (int) sIzq.get(3);
                        bestSol = sIzq;
                    }
                    System.out.println("izq");
                    //System.out.println("izq " + a + " " + mid + " " + sIzq.get(0) + " " + sIzq.get(3) + " " + ((double) sIzq.get(0) / (int) sIzq.get(3)) + " " + remainingTime);
                } else {
                    double r = (double) sDer.get(0) / (int) sDer.get(3);
                    a = mid + 1;
                    if (r > bestRatio) {
                        bestRatio = r;
                        bestK = (int) sDer.get(3);
                        bestSol = sDer;
                    }
                    System.out.println("der");
                    //System.out.println("der " + (mid + 1) + " " + b + " " + sDer.get(0) + " " + sDer.get(3) + " " + ((double) sDer.get(0) / (int) sDer.get(3)) + " " + remainingTime);
                }
            }


    
            // Guardar mejor solución
            for (int p : (Set<Integer>) bestSol.get(1)) {
                selectedOrders.add(p);
            }
            for (int q : (Set<Integer>) bestSol.get(2)) {
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
