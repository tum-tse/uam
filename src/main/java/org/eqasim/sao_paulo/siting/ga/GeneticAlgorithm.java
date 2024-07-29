package org.eqasim.sao_paulo.siting.ga;

import org.eqasim.sao_paulo.siting.Utils.*;

import org.matsim.api.core.v01.Id;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import org.matsim.contrib.dvrp.fleet.ImmutableDvrpVehicleSpecification;
import net.bhl.matsim.uam.infrastructure.UAMStation;
import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import net.bhl.matsim.uam.infrastructure.UAMVehicleType;

import org.apache.log4j.Logger;
import com.google.ortools.Loader;
import com.google.ortools.sat.*;
import com.google.ortools.constraintsolver.*;
import com.google.ortools.constraintsolver.IntVar;
import com.google.ortools.util.Domain;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.*;
import java.util.stream.Collectors;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.io.FileWriter;
import java.util.stream.Stream;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Callable;

public class GeneticAlgorithm {
    private static final Logger log = Logger.getLogger(GeneticAlgorithm.class);

    // Genetic Algorithm parameters ====================================================================================
    private static final int MAX_GENERATIONS = 100; // Max number of generations
    private static final int CROSSOVER_DISABLE_AFTER = 100; // New field to control when to stop crossover
    private static final int POP_SIZE = 50; // Population size
    private static final double MUTATION_RATE = 0.05; // Mutation rate
    private static final double CROSSOVER_RATE = 0.7; // Crossover rate
    private static final int TOURNAMENT_SIZE = 5; // Tournament size for selection

    private static final double ALPHA = - 1; // Weight for changed flight distances
    private static final double BETA = - 1; // Weight for change in travel time
    private static final double BETA_CRUCIAL_TIME_ChANGE = - 0.1; //TODO: need to reconsider the value
    private static final double PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION = -10000;

    private static final long SEED = 4711; // MATSim default Random Seed
    private static final Random rand = new Random(SEED);

    // Parameters and constant for the UAM problem =====================================================================
    private static int FIRST_UAM_VEHICLE_ID = 1;
    private static final int VALUE_FOR_NO_VEHICLE_AVAILABLE = -1; // For example, using -1 as an indicator of no vehicle available for a trip
    private static final double END_SERVICE_TIME_OF_THE_DAY = 3600*36; // End service time of the day
    private static final double VEHICLE_CRUISE_SPEED = 350000.0 / 3600.0; // Vehicle cruise speed in m/s
    private static final int VEHICLE_CAPACITY = 4; // Vehicle capacity

    // Variables for the UAM problem ===================================================================================
    private static final int BUFFER_START_TIME = 3600*7; // Buffer start time for the first trip
    private static final int BUFFER_END_TIME = 3600*7+240; // Buffer end time for the last trip
    private static final double SEARCH_RADIUS_ORIGIN = 1500; // search radius for origin station
    private static final double SEARCH_RADIUS_DESTINATION = 1500; // search radius for destination station

    // Helpers for the UAM problem =====================================================================================
    private static final double THRESHOLD_FOR_TRIPS_LONGER_THAN = SEARCH_RADIUS_ORIGIN;
    private static final String THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING = String.valueOf(THRESHOLD_FOR_TRIPS_LONGER_THAN);
    private static int NUMBER_OF_TRIPS_LONGER_TAHN = 0;

    // Assuming these arrays are initialized elsewhere in your code:
    private static double[] flightDistances; // Distances for each trip
    private static double[][] accessTimesOriginal; // Original access times for each trip
    private static double[][] accessTimesUpdated; // Updated access times for each trip and vehicle
    private static double[][] egressTimesOriginal; // Original egress times for each trip
    private static double[][] egressTimesUpdated; // Updated egress times for each trip and vehicle
    private static double[][] waitingTimes; // Waiting times at the parking station for each trip and vehicle

    // Data container for the UAM problem ==============================================================================
    private static List<UAMTrip> subTrips = null;
    private static Map<Id<UAMStation>, UAMStation> stations = null;
    private static final Map<Id<UAMStation>, List<UAMVehicle>> originStationVehicleMap = new HashMap<>();
    private static final Map<Id<DvrpVehicle>, UAMStation> vehicleOriginStationMap = new ConcurrentHashMap<>();
    private static final Map<Id<DvrpVehicle>, UAMStation> vehicleDestinationStationMap = new ConcurrentHashMap<>();
    private static Map<String, List<UAMVehicle>> tripVehicleMap = new ConcurrentHashMap<>(); // Update tripVehicleMap to use ConcurrentHashMap
    //private static final Map<UAMVehicle, Integer> vehicleOccupancyMap = new HashMap<>();

    // Data container for outputs
    private static final PriorityQueue<SolutionFitnessPair> solutionsHeap = new PriorityQueue<>(Comparator.comparingDouble(SolutionFitnessPair::getFitness));
    private static final PriorityQueue<SolutionFitnessPair> repairedSolutionsHeap = new PriorityQueue<>(Comparator.comparingDouble(SolutionFitnessPair::getFitness));
    private static final Map<String, Double> finalSolutionTravelTimeChanges = new HashMap<>(); // Additional field to store travel time change of each trip for the final best feasible solution
    private static final Map<String, Double> finalSolutionFlightDistanceChanges = new HashMap<>(); // Additional field to store saved flight distance of each trip for the final best feasible solution
    private static final Map<String, Double> finalSolutionDepartureRedirectionRate = new HashMap<>(); // Additional field to store redirection rate of each trip for the final best feasible solution
    private static final Map<String, Double> finalSolutionArrivalRedirectionRate = new HashMap<>(); // Additional field to store redirection rate of each trip for the final best feasible solution
    private static final Map<String, Double> finalSolutionTotalTravelTime = new HashMap<>(); // Additional field to store total travel time of each trip for the final best feasible solution
    private static final Map<String, String> finalSolutionAssignedAccessStation = new HashMap<>(); // Additional field to store assigned access station of each trip for the final best feasible solution
    private static final Map<String, String> finalSolutionAssignedEgressStation = new HashMap<>(); // Additional field to store assigned egress station of each trip for the final best feasible solution

    // Parallel computing
    private static final int numProcessors = Runtime.getRuntime().availableProcessors();
    private static final int bufferDivider = 1;
    // TODO: Create an initial population of solutions using domain-specific knowledge (in our case is the vehicles which were used to create the initial fleet of the vehicles).
    // TODO: How to handle the extremely large travel time?
    // Main method to run the the specifyed algorithm ==================================================================
    public static void main(String[] args) throws IOException, InterruptedException {
        // Load data
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();

        //subTrips = extractSubTrips(dataLoader.getUamTrips());
        String filePath = "/home/tumtse/Documents/haowu/uam/uam/scenarios/1-percent/sao_paulo_population2trips.csv";
        subTrips = readTripsFromCsv(filePath);
        // Randomly select 10% trips from the list of subTrips
        subTrips = subTrips.stream()
                .filter(trip -> rand.nextDouble() < 0.01)
                .collect(Collectors.toCollection(ArrayList::new));

        log.info("The number of UAM trips: " + subTrips.size());

        //vehicles = dataLoader.getVehicles();
        stations = dataLoader.getStations();

        // Initialize the origin station and destination station for each trip
        for (UAMTrip uamTrip : subTrips) {
            uamTrip.setOriginStation(findNearestStation(uamTrip, stations, true));
            uamTrip.setDestinationStation(findNearestStation(uamTrip, stations, false));
        }
        saveStationVehicleNumber(subTrips);
        tripVehicleMap = findNearbyVehiclesToTrips(subTrips);

        SolutionFitnessPair bestFeasibleSolutionFitnessPair = solveWithGA();
        int[] bestFeasibleSolution = bestFeasibleSolutionFitnessPair.getSolution();
        System.out.println("Best feasible solution: " + Arrays.toString(bestFeasibleSolution));
        System.out.println("The fitness of the best feasible solution: " + bestFeasibleSolutionFitnessPair.getFitness());

        // Calculate and print the performance indicators
        calculateFitness(bestFeasibleSolution, true);
        printPerformanceIndicators(bestFeasibleSolution, "src/main/java/org/eqasim/sao_paulo/siting/ga/trip_statistics.csv");

        // Print the NUMBER_OF_TRIPS_LONGER_THAN
        System.out.println("Threshold for trips longer than " + THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING + ": " + NUMBER_OF_TRIPS_LONGER_TAHN);
    }

    // CP solver 1 =====================================================================================================
    private static SolutionFitnessPair solveWithCP1() {
        Solver solver = new Solver("SolveUAMTripAssignment");
        solver.makeTimeLimit(1800*1000);

        int numTrips = subTrips.size();
        List<Integer> vehicleIds = tripVehicleMap.values().stream()
                .flatMap(List::stream)
                .map(v -> Integer.parseInt(v.getId().toString()))
                .distinct()
                .collect(Collectors.toList());
        int numVehicles = vehicleIds.size();

        // Variables: Trip assignment for each vehicle
        IntVar[][] vehicleTripsAssignments = new IntVar[numVehicles][numTrips];
        for (int v = 0; v < numVehicles; v++) {
            for (int t = 0; t < numTrips; t++) {
                vehicleTripsAssignments[v][t] = solver.makeIntVar(0, 1, "vehicle_" + v + "_trip_" + t);
            }
        }

        // Constraints: Each trip must be assigned to exactly one vehicle
        for (int t = 0; t < numTrips; t++) {
            IntVar[] tripAssignments = new IntVar[numVehicles];
            for (int v = 0; v < numVehicles; v++) {
                tripAssignments[v] = vehicleTripsAssignments[v][t];
            }
            solver.addConstraint(solver.makeEquality(solver.makeSum(tripAssignments), 1));
        }

        // Constraints: Each vehicle cannot exceed its capacity
        for (int v = 0; v < numVehicles; v++) {
            IntVar vehicleLoad = solver.makeSum(vehicleTripsAssignments[v]).var();
            solver.addConstraint(solver.makeLessOrEqual(vehicleLoad, VEHICLE_CAPACITY));
        }

        // Objective: Minimize the fitness function using getFitnessPerVehicle
        IntVar objective = calculateFitnessForCP(solver, vehicleTripsAssignments, vehicleIds);
        OptimizeVar mObjective = solver.makeMaximize(objective, 1);

        // Flatten the IntVar[][] array into IntVar[] for the search phase
        IntVar[] flattenedAssignments = Arrays.stream(vehicleTripsAssignments).flatMap(Arrays::stream).toArray(IntVar[]::new);

        // Set up the search parameters
        DecisionBuilder db = solver.makePhase(flattenedAssignments, Solver.CHOOSE_FIRST_UNBOUND, Solver.ASSIGN_MIN_VALUE);
        solver.newSearch(db, mObjective);

        SolutionFitnessPair bestFeasibleSolutionFitnessPair = null;

        if (solver.nextSolution()) {
            int[] bestSolution = new int[numTrips];
            for (int t = 0; t < numTrips; t++) {
                for (int v = 0; v < numVehicles; v++) {
                    if (vehicleTripsAssignments[v][t].value() == 1) {
                        bestSolution[t] = vehicleIds.get(v);
                    }
                }
            }
            double bestFitness = calculateFitness(bestSolution, false);
            bestFeasibleSolutionFitnessPair = new SolutionFitnessPair(bestSolution, bestFitness);
        }

        solver.endSearch();
        return bestFeasibleSolutionFitnessPair;
    }
    private static IntVar calculateFitnessForCP(Solver solver, IntVar[][] vehicleTripsAssignments, List<Integer> vehicleIds) {
        int numTrips = vehicleTripsAssignments[0].length;
        int numVehicles = vehicleTripsAssignments.length;
        IntVar fitness = solver.makeIntVar(Integer.MIN_VALUE, Integer.MAX_VALUE, "fitness"); //TODO: need to specify the range.

        // Sum up the fitness contributions for each vehicle
        for (int v = 0; v < numVehicles; v++) {
            // Accumulate the fitness for each vehicle
            IntVar vehicleFitness = solver.makeIntVar(-1000000, 1000000, "vehicle_fitness_" + v); //TODO: need to specify the range.

            for (int t = 0; t < numTrips; t++) {
                // Create a map of vehicle assignments
                Map<Integer, List<UAMTrip>> vehicleAssignments = new HashMap<>();
                vehicleAssignments.put(vehicleIds.get(v), new ArrayList<>());

                if (vehicleTripsAssignments[v][t].bound() && vehicleTripsAssignments[v][t].value() == 1) {
                    vehicleAssignments.get(vehicleIds.get(v)).add(subTrips.get(t));
                }

                double fitnessContribution = getFitnessPerVehicle(false, vehicleAssignments, 0.0);
                IntVar fitnessContributionVar = solver.makeIntConst((int) fitnessContribution);
                vehicleFitness = solver.makeSum(vehicleFitness, fitnessContributionVar).var();
            }

            fitness = solver.makeSum(fitness, vehicleFitness).var();
        }

        return fitness;
    }

    // CP solver 2 =====================================================================================================
    private static SolutionFitnessPair solveWithCP2() {
        Loader.loadNativeLibraries();

        CpModel model = new CpModel();

        // Create variables
        int numTrips = subTrips.size();

        // Variables: Vehicle assignment for each trip
        com.google.ortools.sat.IntVar[] vehicleAssignments = new com.google.ortools.sat.IntVar[numTrips];
        for (int i = 0; i < numTrips; i++) {
            UAMTrip trip = subTrips.get(i);
            List<UAMVehicle> availableVehicles = tripVehicleMap.get(trip.getTripId());
            long[] availableVehicleIds = availableVehicles.stream().mapToLong(v -> Long.parseLong(v.getId().toString())).toArray();

            // Define the domain of the variable to be the available vehicles for this trip
            vehicleAssignments[i] = model.newIntVarFromDomain(Domain.fromValues(availableVehicleIds), "vehicle_" + i);
        }


        // Constraints: Each vehicle cannot exceed its capacity
        for (int vehicleId : tripVehicleMap.values().stream().flatMap(List::stream).mapToInt(v -> Integer.parseInt(v.getId().toString())).distinct().toArray()) {
            com.google.ortools.sat.IntVar vehicleCapacityUsage = model.newIntVar(0, VEHICLE_CAPACITY, "vehicleCapacityUsage_" + vehicleId);
            com.google.ortools.sat.IntVar[] vehicleLoad = new com.google.ortools.sat.IntVar[numTrips];
            for (int i = 0; i < numTrips; i++) {
                vehicleLoad[i] = model.newIntVar(0, 1, "vehicleLoad_" + i + "_" + vehicleId);
                model.addEquality(vehicleLoad[i], model.newConstant(vehicleAssignments[i].getIndex()));
            }

            // Create a list of indices of the vehicleLoad variables
            int[] vehicleLoadIndices = Arrays.stream(vehicleLoad).mapToInt(com.google.ortools.sat.IntVar::getIndex).toArray();
            model.addEquality(vehicleCapacityUsage, model.newConstant(Arrays.stream(vehicleLoadIndices).sum()));
            model.addLessOrEqual(vehicleCapacityUsage, VEHICLE_CAPACITY);
        }


        // Create solver and set parameters
        CpSolver solver = new CpSolver();
        solver.getParameters().setMaxTimeInSeconds(3600.0);
        solver.getParameters().setNumWorkers(1);

        // Solve the problem
        CpSolverStatus status = solver.solve(model);

        // Objective: maximize the fitness
        // Create a variable to hold the total fitness and set it as the objective
        com.google.ortools.sat.IntVar totalFitness = model.newIntVar(Integer.MIN_VALUE, Integer.MAX_VALUE, "totalFitness");
        LinearExprBuilder fitnessExpr = LinearExpr.newBuilder();

        // Use getFitnessPerVehicle logic to create the fitness expression
        Map<Integer, List<UAMTrip>> vehicleAssignmentsMap = new HashMap<>();
        for (int i = 0; i < numTrips; i++) {
            int vehicleId = (int) solver.value(vehicleAssignments[i]);
            vehicleAssignmentsMap.putIfAbsent(vehicleId, new ArrayList<>());
            vehicleAssignmentsMap.get(vehicleId).add(subTrips.get(i));
        }
        long fitness = (long) getFitnessPerVehicle(false, vehicleAssignmentsMap, 0.0);
        fitnessExpr.addTerm(totalFitness, fitness);

        model.maximize(fitnessExpr);

        if (status == CpSolverStatus.OPTIMAL || status == CpSolverStatus.FEASIBLE) {
            // Extract the best solution
            int[] bestSolution = new int[numTrips];
            for (int i = 0; i < numTrips; i++) {
                bestSolution[i] = (int) solver.value(vehicleAssignments[i]);
            }
            double bestFitness = solver.value(totalFitness);
            return new SolutionFitnessPair(bestSolution, bestFitness);
        } else {
            throw new IllegalStateException("No feasible solution found within the time limit.");
        }

    }

    // GA solver =======================================================================================================
    private static SolutionFitnessPair solveWithGA() throws InterruptedException {
        // Initialize population and solutions heap in the first generation
        int[][] population = initializePopulation();

        // GA iterations
        for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
            if(gen > 0){
                population = evolvePopulation(population, gen);
            }
            PriorityQueue<SolutionFitnessPair> iterationHeap = updateSolutionsHeap(population);
            System.out.println("Generation " + gen + ": Best fitness = " + iterationHeap.peek().getFitness());
        }

        // Repair infeasible solutions using Simulated Annealing
        // Executor service and queue for parallel processing
        ExecutorService executorService = Executors.newFixedThreadPool(numProcessors);
        ArrayBlockingQueue<SolutionFitnessPair> queue = new ArrayBlockingQueue<>(solutionsHeap.size());
        repairInfeasibleSolutions(numProcessors, executorService, queue);
        // Make sure that the file is not written before all threads are finished
        while (!executorService.isTerminated())
            Thread.sleep(200);

        // Find the best feasible solution at the end of GA execution without altering the original solutions heap
        SolutionFitnessPair bestFeasibleSolutionFitnessPair = findBestFeasibleSolution();
        return bestFeasibleSolutionFitnessPair;
    }

    // Initialize population with random assignments
    private static int[][] initializePopulation() {
        int[][] population = new int[GeneticAlgorithm.POP_SIZE][];
        for (int i = 0; i < GeneticAlgorithm.POP_SIZE; i++) {
            population[i] = generateIndividual();
        }
        return population;
    }
    // Generate a random individual
    private static int[] generateIndividual() {
        int[] individual = new int[subTrips.size()];
        //resetVehicleCapacities(tripVehicleMap); // Reset the vehicle capacity since capacity of vehicles will be updated during each individual generation
        //resetVehicleOccupancy(vehicleOccupancyMap);
        for (int i = 0; i < individual.length; i++) {
            assignAvailableVehicle(i, individual);
        }
        return individual;
    }

    // Modified evolvePopulation method
    private static int[][] evolvePopulation(int[][] population, int currentGeneration) {
        int[][] newPop = new int[POP_SIZE][];
        boolean useCrossover = currentGeneration < CROSSOVER_DISABLE_AFTER;

        for (int i = 0; i < POP_SIZE; i++) {
            int[] parent1 = select(population);
            int[] parent2 = select(population);
            int[] child;

            if (useCrossover && rand.nextDouble() < CROSSOVER_RATE) {
                child = crossover(parent1, parent2);
            } else {
                child = rand.nextBoolean() ? parent1 : parent2; // Skip crossover, use parent directly
            }

            newPop[i] = mutate(child); // Mutation is always applied
        }
        return newPop;
    }
    // Selection - Tournament selection with null check
    private static int[] select(int[][] pop) {
        int[] best = null;
        double bestFitness = -Double.MAX_VALUE;

        for (int i = 0; i < TOURNAMENT_SIZE; i++) {
            int[] individual = pop[rand.nextInt(POP_SIZE)];
            double fitness = calculateFitness(individual, false);
            if (fitness > bestFitness) {
                best = individual;
                bestFitness = fitness;
            }
        }

        //TODO: Reconsider this part
        // Check if 'best' is null which can be due to empty population array
        if (best == null) {
            // Handle the scenario when no best individual was found
            // For example: return a new random individual or handle the error
            // Returning new random individual as a fallback:
            best = generateIndividual();  // Assuming generateIndividual() creates a valid random individual

            //throw new IllegalArgumentException("No best individual found");
            throw new IllegalArgumentException("No best individual found");
        }

        return Arrays.copyOf(best, best.length); // Return a copy of the best individual
    }
    //TODO: consider to repair the solution if it is not feasible due to the violation of vehicle capacity constraint
    // Crossover - Single point crossover //TODO: Implement other types of crossover instead of single point
    private static int[] crossover(int[] parent1, int[] parent2) {
        int[] child = new int[parent1.length];

        int crossoverPoint = rand.nextInt(parent1.length);
        for (int i = 0; i < crossoverPoint; i++) {
            child[i] = parent1[i];
        }
        for (int i = crossoverPoint; i < parent2.length; i++) {
            child[i] = parent2[i];
        }
        return child;
    }
    // Mutation - Randomly change vehicle assignment
    private static int[] mutate(int[] individual) {
        //resetVehicleCapacities(tripVehicleMap); // Reset the vehicle capacity since capacity of vehicles will be updated during each individual generation
        //resetVehicleOccupancy(vehicleOccupancyMap);
        for (int i = 0; i < individual.length; i++) {
            if (rand.nextDouble() < MUTATION_RATE) {
                assignAvailableVehicle(i, individual);
            }
        }
        return individual;
    }

    // Method to assign an available vehicle to a trip
    private static void assignAvailableVehicle(int i, int[] individual) {
        UAMTrip trip = subTrips.get(i);
        List<UAMVehicle> vehicleList = tripVehicleMap.get(trip.getTripId());

        /*        //add occupancy constraint
        if (!vehicleList.isEmpty()) {
            Iterator<UAMVehicle> iterator0 = vehicleList.iterator();
            while (iterator0.hasNext()) {
                UAMVehicle vehicle = iterator0.next();
                int occupancy = vehicleOccupancyMap.get(vehicle);
                if (occupancy <= 0) {
                    iterator0.remove();
                }
            }
        }*/

        //synchronize the block that checks for empty vehicle list and adds a new vehicle (i.e., vehicle has capacity) after checking the egress constraint
        synchronized (tripVehicleMap) {
                // Add a new vehicle for the trip when there is no available vehicle
            if (vehicleList==null||vehicleList.isEmpty()) {
                if(vehicleList==null){
                    vehicleList = new ArrayList<>();
                }
                UAMVehicle vehicle = feedDataForVehicleCreation(trip, false);
                vehicleList.add(vehicle);
                tripVehicleMap.put(trip.getTripId(), vehicleList);
                //vehicleOccupancyMap.put(vehicle, VEHICLE_CAPACITY);
            }
        }

        if (!vehicleList.isEmpty()) {

            //add access constraint
            int vehicleIndex = rand.nextInt(vehicleList.size());
            UAMVehicle selectedVehicle = vehicleList.get(vehicleIndex);
            //Integer currentCapacity = vehicleOccupancyMap.get(selectedVehicle);

            individual[i] = Integer.parseInt(selectedVehicle.getId().toString());

            // Decrement capacity and explicitly update tripVehicleMap
            //vehicleOccupancyMap.put(selectedVehicle, vehicleOccupancyMap.get(selectedVehicle) - 1);

        } else {
            // Handle the case when there is no available vehicle: This might involve setting a default value or handling it in the fitness function
            individual[i] = VALUE_FOR_NO_VEHICLE_AVAILABLE;
            throw new IllegalArgumentException("Need to handle the case when there is no available vehicle for the trip.");
        }
    }

    // Objective function ==============================================================================================
    // Calculate fitness for an individual
    private static double calculateFitness(int[] individual, boolean isFinalBestFeasibleSolution) {
        double fitness = 0.0;
        Map<Integer, List<UAMTrip>> vehicleAssignments = new HashMap<>();

        // Organize trips by assigned vehicle
        for (int i = 0; i < individual.length; i++) {
            int vehicleId = individual[i];
            if (!vehicleAssignments.containsKey(vehicleId)) {
                vehicleAssignments.put(vehicleId, new ArrayList<>());
            }
            vehicleAssignments.get(vehicleId).add(subTrips.get(i));
        }

        if (isFinalBestFeasibleSolution){
            int pooledTrips = 0;
            for (List<UAMTrip> trips : vehicleAssignments.values()) {
                if (trips.size() > 1) {
                    pooledTrips += trips.size(); // Count all pooled trips
                }
            }
            double poolingRate = (double) pooledTrips / subTrips.size();
            System.out.println("Pooling rate: " + poolingRate);
        }

        fitness = getFitnessPerVehicle(isFinalBestFeasibleSolution, vehicleAssignments, fitness);

        return fitness;
    }
    private static double getFitnessPerVehicle(boolean isFinalBestFeasibleSolution, Map<Integer, List<UAMTrip>> vehicleAssignments, double fitness) {
        // Calculate fitness per vehicle
        for (Map.Entry<Integer, List<UAMTrip>> entry : vehicleAssignments.entrySet()) {
            List<UAMTrip> trips = entry.getValue();
            UAMStation originStationOfVehicle = vehicleOriginStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));
            UAMStation destinationStationOfVehicle = vehicleDestinationStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));

            // safety check
            if (trips.isEmpty()) continue;
            if (trips.size() == 1){
                UAMTrip trip = trips.get(0);
                fitness = getFitnessForNonPooledOrBaseTrip(trip, originStationOfVehicle, destinationStationOfVehicle, fitness, isFinalBestFeasibleSolution);
                continue;
            }

            // Find the base trip (the trip with the latest arrival time at the departure UAM station)
            UAMTrip baseTrip = trips.get(0);
            for (UAMTrip trip : trips) {
                double accessTimeOfBaseTrip = baseTrip.calculateAccessTeleportationTime(originStationOfVehicle);
                double accessTimeOfPooledTrip = trip.calculateAccessTeleportationTime(originStationOfVehicle);
                if ((trip.getDepartureTime() + accessTimeOfPooledTrip) > (baseTrip.getDepartureTime() + accessTimeOfBaseTrip)) {
                    baseTrip = trip;
                }
            }

            double boardingTimeForAllTrips = baseTrip.getDepartureTime() + baseTrip.calculateAccessTeleportationTime(originStationOfVehicle);
            // Calculate fitness based on the proposed pooling option
            for (UAMTrip trip : trips) {
                if(trip.getTripId().equals(baseTrip.getTripId())){
                    fitness = getFitnessForNonPooledOrBaseTrip(trip, originStationOfVehicle, destinationStationOfVehicle, fitness, isFinalBestFeasibleSolution);
                    continue;
                }

                double tripTimeChange = 0.0;
                double tripFlightDistanceChange = 0.0;
                double tripTotalTravelTime = 0.0;

                // calculate change in flight distance
                double flightDistanceChange = getFlightDistanceChange(trip, originStationOfVehicle, destinationStationOfVehicle);
                fitness += ALPHA * flightDistanceChange;
                tripFlightDistanceChange += flightDistanceChange;
                // calculate saved flight distance
                double savedFlightDistance = trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation());
                fitness += ALPHA * (-1) * savedFlightDistance;
                tripFlightDistanceChange += savedFlightDistance;
                if(isFinalBestFeasibleSolution){
                    finalSolutionFlightDistanceChanges.put(trip.getTripId(), tripFlightDistanceChange);
                }
                // calculate change in flight time due to the change in flight distance
                double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
                fitness += BETA * flightTimeChange;
                tripTimeChange += flightTimeChange;
                // calculate additional travel time
                double originalArrivalTimeForThePooledTrip = trip.getDepartureTime() + trip.calculateAccessTeleportationTime(trip.getOriginStation());
                double travelTimeChangeDueToAccessMatching = boardingTimeForAllTrips - originalArrivalTimeForThePooledTrip;
                tripTimeChange += travelTimeChangeDueToAccessMatching;
                if(travelTimeChangeDueToAccessMatching > 0) {
                    fitness += BETA * travelTimeChangeDueToAccessMatching;
                } else {
                    fitness += BETA * (- travelTimeChangeDueToAccessMatching); //TODO: reconsider for "negative additional travel time" cases
                }
                double additionalTravelTimeDueToEgressMatching = getTravelTimeChangeDueToEgressMatching(trip, destinationStationOfVehicle);
                fitness += BETA * additionalTravelTimeDueToEgressMatching;
                tripTimeChange += additionalTravelTimeDueToEgressMatching;
                if(isFinalBestFeasibleSolution){
                    finalSolutionTravelTimeChanges.put(trip.getTripId(), tripTimeChange);
                }
                if(isFinalBestFeasibleSolution){
                    double departureRedirectionRate = ( trip.calculateAccessTeleportationDistance(originStationOfVehicle) - trip.calculateAccessTeleportationDistance(trip.getOriginStation()) ) / ( trip.calculateAccessTeleportationDistance(trip.getOriginStation()) );
                    finalSolutionDepartureRedirectionRate.put(trip.getTripId(), departureRedirectionRate);
                    double arrivalRedirectionRate = ( trip.calculateEgressTeleportationDistance(destinationStationOfVehicle) - trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) ) / ( trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) );
                    finalSolutionArrivalRedirectionRate.put(trip.getTripId(), arrivalRedirectionRate);

                    // total travel time for the trip
                    //TODO: Should the accessTime = boardingTimeForAllTrips - trip.getDepartureTime()?
                    tripTotalTravelTime = trip.calculateAccessTeleportationTime(originStationOfVehicle) + trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) / VEHICLE_CRUISE_SPEED + trip.calculateEgressTeleportationTime(destinationStationOfVehicle);
                    finalSolutionTotalTravelTime.put(trip.getTripId(), tripTotalTravelTime);

                    // assigned origin station
                    finalSolutionAssignedAccessStation.put(trip.getTripId(), originStationOfVehicle.getId().toString());
                    // assigned destination station
                    finalSolutionAssignedEgressStation.put(trip.getTripId(), destinationStationOfVehicle.getId().toString());
                }
            }
            //add penalty for the case when vehicle capacity is violated
            if(trips.size()>VEHICLE_CAPACITY){
                fitness += PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION * (trips.size()-VEHICLE_CAPACITY);
            }
        }
        return fitness;
    }
    private static double getFitnessForNonPooledOrBaseTrip(UAMTrip trip, UAMStation originStationOfVehicle, UAMStation destinationStationOfVehicle, double fitness, boolean isFinalBestFeasibleSolution) {
        double tripTimeChange = 0.0;
        double tripFlightDistanceChange = 0.0;

        // calculate change in flight distance
        double flightDistanceChange = getFlightDistanceChange(trip, originStationOfVehicle, destinationStationOfVehicle);
        fitness += ALPHA * flightDistanceChange;
        tripFlightDistanceChange += flightDistanceChange;
        if(isFinalBestFeasibleSolution){
            finalSolutionFlightDistanceChanges.put(trip.getTripId(), tripFlightDistanceChange);
        }
        // calculate change in flight time due to the change in flight distance
        double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
        fitness += BETA * flightTimeChange;
        tripTimeChange += flightTimeChange;
        // calculate change in travel time due to access matching
        double travelTimeChangeDueToAccessMatching = trip.calculateAccessTeleportationTime(originStationOfVehicle) - trip.calculateAccessTeleportationTime(trip.getOriginStation());
        tripTimeChange += travelTimeChangeDueToAccessMatching;
        if(travelTimeChangeDueToAccessMatching > 0) {
            fitness += BETA * travelTimeChangeDueToAccessMatching;
        } else {
            fitness += BETA * (- travelTimeChangeDueToAccessMatching);
        }
        // calculate change in travel time due to egress matching
        double travelTimeChangeDueToEgressMatching = getTravelTimeChangeDueToEgressMatching(trip, destinationStationOfVehicle);
        fitness += BETA * travelTimeChangeDueToEgressMatching;
        tripTimeChange += travelTimeChangeDueToEgressMatching;
        if(isFinalBestFeasibleSolution){
            finalSolutionTravelTimeChanges.put(trip.getTripId(), tripTimeChange);
        }
        if(isFinalBestFeasibleSolution){
            double departureRedirectionRate = ( trip.calculateAccessTeleportationDistance(originStationOfVehicle) - trip.calculateAccessTeleportationDistance(trip.getOriginStation()) ) / ( trip.calculateAccessTeleportationDistance(trip.getOriginStation()) );
            finalSolutionDepartureRedirectionRate.put(trip.getTripId(), departureRedirectionRate);
            double arrivalRedirectionRate = ( trip.calculateEgressTeleportationDistance(destinationStationOfVehicle) - trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) ) / ( trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) );
            finalSolutionArrivalRedirectionRate.put(trip.getTripId(), arrivalRedirectionRate);

            // total travel time for the trip
            double totalTravelTime = trip.calculateAccessTeleportationTime(originStationOfVehicle) + trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) / VEHICLE_CRUISE_SPEED + trip.calculateEgressTeleportationTime(destinationStationOfVehicle);
            finalSolutionTotalTravelTime.put(trip.getTripId(), totalTravelTime);

            // assigned origin station
            finalSolutionAssignedAccessStation.put(trip.getTripId(), originStationOfVehicle.getId().toString());
            // assigned destination station
            finalSolutionAssignedEgressStation.put(trip.getTripId(), destinationStationOfVehicle.getId().toString());
        }
        return fitness;
    }
    private static double getFlightDistanceChange(UAMTrip trip, UAMStation originStationOfVehicle, UAMStation destinationStationOfVehicle) {
        return trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) - trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation());
    }
    private static double getTravelTimeChangeDueToEgressMatching(UAMTrip trip, UAMStation destinationStationOfVehicle) {
        return trip.calculateEgressTeleportationTime(destinationStationOfVehicle) - trip.calculateEgressTeleportationTime(trip.getDestinationStation());
    }

    // Helper methods for GA ===========================================================================================
    private static void resetVehicleOccupancy(Map<UAMVehicle, Integer> vehicleOccupancyMap) {
        for (Map.Entry<UAMVehicle, Integer> entry : vehicleOccupancyMap.entrySet()) {
            entry.setValue(VEHICLE_CAPACITY);
        }
    }
    private static Map<String, List<UAMVehicle>> findNearbyVehiclesToTrips(List<UAMTrip> subTrips) {
        Map<String, List<UAMVehicle>> tripVehicleMap = new HashMap<>();
        for (UAMTrip trip : subTrips) {
            for (UAMStation station : stations.values()) {
                if (trip.calculateAccessTeleportationDistance(station) <= SEARCH_RADIUS_ORIGIN) {
                    List<UAMVehicle> vehicles = originStationVehicleMap.get(station.getId());
                    if (vehicles == null){
                        continue;
                    }
                    List<UAMVehicle> existingVehicles = tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>());

                    //add egress constraint
                    vehicles = vehicles.stream()
                            .filter(vehicle -> trip.calculateEgressTeleportationDistance(vehicleDestinationStationMap.get(vehicle.getId())) <= SEARCH_RADIUS_DESTINATION)
                            .collect(Collectors.toCollection(ArrayList::new));

                    existingVehicles.addAll(vehicles);

                    tripVehicleMap.put(trip.getTripId(), existingVehicles);
                } /*else {
                    if (trip.calculateAccessTeleportationDistance(station)> THRESHOLD_FOR_TRIPS_LONGER_THAN){
                        NUMBER_OF_TRIPS_LONGER_TAHN++;
                    }

                    // ----- Add a new vehicle for the trip when the access teleportation distance is longer than the search radius
                    List<UAMVehicle> vehicleList = tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>());
                    UAMVehicle vehicle = feedDataForVehicleCreation(trip, false);
                    vehicleList.add(vehicle);
                    tripVehicleMap.put(trip.getTripId(), vehicleList);
                    //vehicleOccupancyMap.put(vehicle, VEHICLE_CAPACITY);
                }*/
            }
        }
        return tripVehicleMap;
    }

    // SolutionFitnessPair related methods =============================================================================
    // SolutionFitnessPair class to hold individual solutions and their fitness
    private static class SolutionFitnessPair {
        private final int[] solution;
        private final double fitness;

        public SolutionFitnessPair(int[] solution, double fitness) {
            this.solution = solution;
            this.fitness = fitness;
        }

        public int[] getSolution() {
            return solution;
        }

        public double getFitness() {
            return fitness;
        }
    }
    // New method to update the solutions heap without side effects
    private static PriorityQueue<SolutionFitnessPair> updateSolutionsHeap(int[][] population) throws InterruptedException {
        PriorityQueue<SolutionFitnessPair> iterationHeap = new PriorityQueue<>(Comparator.comparingDouble(SolutionFitnessPair::getFitness).reversed());

        // Set up ExecutorService and ThreadCounter
        ExecutorService executorService = Executors.newFixedThreadPool(numProcessors);
        ThreadCounter threadCounter = new ThreadCounter();

        // Create a list to hold Future objects
        List<Future<SolutionFitnessPair>> futures = new ArrayList<>();

        for (int[] individual : population) {
            while (threadCounter.getProcesses() >= numProcessors - 1) //TODO: could be adjusted to a larger number when number of trips is small!
                Thread.sleep(200);

            // Submit FitnessCalculator tasks
            futures.add(executorService.submit(new FitnessCalculator(individual, threadCounter)));
        }

        // Collect results
        for (Future<SolutionFitnessPair> future : futures) {
            try {
                SolutionFitnessPair solutionPair = future.get();
                if (solutionsHeap.size() == POP_SIZE) {
                    if (solutionPair.getFitness() > solutionsHeap.peek().getFitness()) {
                        solutionsHeap.poll();
                        solutionsHeap.add(solutionPair);
                    }
                } else {
                    solutionsHeap.add(solutionPair);
                }
                iterationHeap.add(solutionPair);
            } catch (ExecutionException e) {
                e.printStackTrace();
            }
        }

        // Shutdown ExecutorService and wait for all tasks to finish
        executorService.shutdown();
        while (!executorService.isTerminated())
            Thread.sleep(200);

        return iterationHeap;
    }
    static class FitnessCalculator implements Callable<SolutionFitnessPair> {
        private final int[] individual;
        private final ThreadCounter threadCounter;

        public FitnessCalculator(int[] individual, ThreadCounter threadCounter) {
            this.individual = individual;
            this.threadCounter = threadCounter;
        }

        @Override
        public SolutionFitnessPair call() {
            threadCounter.register();
            try {
                double fitness = GeneticAlgorithm.calculateFitness(individual, false);
                return new SolutionFitnessPair(individual, fitness);
            } finally {
                threadCounter.deregister();
            }
        }
    }
    // Method to find the first feasible solution from the priority queue without altering the original heap
    private static SolutionFitnessPair findBestFeasibleSolution() {
        // Create a new priority queue that is a copy of the original but sorted in descending order by fitness
        PriorityQueue<SolutionFitnessPair> solutionsHeapCopy = new PriorityQueue<>(
                Comparator.comparingDouble(SolutionFitnessPair::getFitness).reversed()
        );
        solutionsHeapCopy.addAll(repairedSolutionsHeap);

        // Iterate through the copied solutions heap to find a feasible solution
        while (!solutionsHeapCopy.isEmpty()) {
            SolutionFitnessPair solutionPair = solutionsHeapCopy.poll(); // Remove and retrieve the solution with the highest fitness
            int[] candidateSolution = solutionPair.getSolution();
            if (isFeasible(candidateSolution, false)) {
                return solutionPair;
            }
        }
        throw new IllegalStateException("No feasible solution found in the entire population");
    }
    // Helper method to check if a solution violates vehicle capacity constraints
    private static boolean isFeasible(int[] solution, boolean isPrintCapacityViolation) {
        boolean isFeasible = true;
        int vehicleCapacityViolated = 0;
        Map<Integer, Integer> vehicleLoadCount = new HashMap<>();
        for (int vehicleId : solution) {
            vehicleLoadCount.put(vehicleId, vehicleLoadCount.getOrDefault(vehicleId, 0) + 1);
        }
        for (int load : vehicleLoadCount.values()) {
            if (load > VEHICLE_CAPACITY) {
                isFeasible = false;
                vehicleCapacityViolated++;
            }
        }
        if(isPrintCapacityViolation){
            System.out.println("Number of vehicles with capacity violation: " + vehicleCapacityViolated);
        }
        return isFeasible;
    }

    // Performance indicators ==========================================================================================
    // Method to calculate and print the performance indicators
    private static void printPerformanceIndicators(int[] solution, String tripStatisticsCSVFile) {
        // Method to calculate and print the number of vehicles by capacity
        Map<Integer, Integer> capacityCount = countVehicleCapacities(solution);
        Set<Integer> uniqueVehicles = new HashSet<>();
        for (int vehicleId : solution) {
            uniqueVehicles.add(vehicleId);
        }
        int totalVehicles = uniqueVehicles.size();

        System.out.println("Vehicle Capacity Rates:");
        for (int capacity = 0; capacity <= VEHICLE_CAPACITY; capacity++) {
            int count = capacityCount.getOrDefault(capacity, 0);
            double rate = (double) count / totalVehicles;
            System.out.println("Capacity " + capacity + ": " + count + " vehicles, Rate: " + rate);
        }

        // Collect travel time changes only for trips assigned to a vehicle shared with others
        List<Double> sharedTravelTimeChanges = new ArrayList<>();
        for (int vehicleId : uniqueVehicles) {
            List<Integer> tripsForVehicle = new ArrayList<>();
            for (int i = 0; i < solution.length; i++) {
                if (solution[i] == vehicleId) {
                    tripsForVehicle.add(i);
                }
            }
            if (tripsForVehicle.size() > 1) { // Vehicle shared with others
                for (int tripIndex : tripsForVehicle) {
                    String tripId = subTrips.get(tripIndex).getTripId();
                    if (finalSolutionTravelTimeChanges.containsKey(tripId)) {
                        sharedTravelTimeChanges.add(finalSolutionTravelTimeChanges.get(tripId));
                    }
                }
            }
        }

        List<Double> sortedTravelTimeChanges = new ArrayList<>(sharedTravelTimeChanges);
        Collections.sort(sortedTravelTimeChanges);

        double averageTravelTime = sortedTravelTimeChanges.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);
        double percentile5thTravelTime = sortedTravelTimeChanges.isEmpty() ? Double.NaN : sortedTravelTimeChanges.get((int) (0.05 * sortedTravelTimeChanges.size()));
        double percentile95thTravelTime = sortedTravelTimeChanges.isEmpty() ? Double.NaN : sortedTravelTimeChanges.get((int) (0.95 * sortedTravelTimeChanges.size()) - 1);

        System.out.println("Average travel time change (shared vehicles): " + averageTravelTime);
        System.out.println("5th percentile of travel time change (shared vehicles): " + percentile5thTravelTime);
        System.out.println("95th percentile of travel time change (shared vehicles): " + percentile95thTravelTime);

        Collection<Double> flightDistanceChanges = finalSolutionFlightDistanceChanges.values();
        List<Double> sortedFlightDistanceChanges = new ArrayList<>(flightDistanceChanges);
        Collections.sort(sortedFlightDistanceChanges);

        double averageFlightDistance = sortedFlightDistanceChanges.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);
        double percentile5thFlightDistance = sortedFlightDistanceChanges.isEmpty() ? Double.NaN : sortedFlightDistanceChanges.get((int) (0.05 * sortedFlightDistanceChanges.size()));
        double percentile95thFlightDistance = sortedFlightDistanceChanges.isEmpty() ? Double.NaN : sortedFlightDistanceChanges.get((int) (0.95 * sortedFlightDistanceChanges.size()) - 1);

        System.out.println("Average flight distance change: " + averageFlightDistance);
        System.out.println("5th percentile of flight distance change: " + percentile5thFlightDistance);
        System.out.println("95th percentile of flight distance change: " + percentile95thFlightDistance);

        Collection<Double> departureRedirectionRates = finalSolutionDepartureRedirectionRate.values();
        List<Double> sortedDepartureRedirectionRates = new ArrayList<>(departureRedirectionRates);
        Collections.sort(sortedDepartureRedirectionRates);

        double averageDepartureRedirectionRate = sortedDepartureRedirectionRates.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);
        double percentile5thDepartureRedirectionRate = sortedDepartureRedirectionRates.isEmpty() ? Double.NaN : sortedDepartureRedirectionRates.get((int) (0.05 * sortedDepartureRedirectionRates.size()));
        double percentile95thDepartureRedirectionRate = sortedDepartureRedirectionRates.isEmpty() ? Double.NaN : sortedDepartureRedirectionRates.get((int) (0.95 * sortedDepartureRedirectionRates.size()) - 1);

        System.out.println("Average departure redirection rate: " + averageDepartureRedirectionRate);
        System.out.println("5th percentile of departure redirection rate: " + percentile5thDepartureRedirectionRate);
        System.out.println("95th percentile of departure redirection rate: " + percentile95thDepartureRedirectionRate);

        Collection<Double> arrivalRedirectionRates = finalSolutionArrivalRedirectionRate.values();
        List<Double> sortedArrivalRedirectionRates = new ArrayList<>(arrivalRedirectionRates);
        Collections.sort(sortedArrivalRedirectionRates);

        double averageArrivalRedirectionRate = sortedArrivalRedirectionRates.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);
        double percentile5thArrivalRedirectionRate = sortedArrivalRedirectionRates.isEmpty() ? Double.NaN : sortedArrivalRedirectionRates.get((int) (0.05 * sortedArrivalRedirectionRates.size()));
        double percentile95thArrivalRedirectionRate = sortedArrivalRedirectionRates.isEmpty() ? Double.NaN : sortedArrivalRedirectionRates.get((int) (0.95 * sortedArrivalRedirectionRates.size()) - 1);

        System.out.println("Average arrival redirection rate: " + averageArrivalRedirectionRate);
        System.out.println("5th percentile of arrival redirection rate: " + percentile5thArrivalRedirectionRate);
        System.out.println("95th percentile of arrival redirection rate: " + percentile95thArrivalRedirectionRate);

        // Print statistics to CSV
        printStatisticsToCsv(solution, tripStatisticsCSVFile);
    }
    // Method to print statistics to a CSV file
    private static void printStatisticsToCsv(int[] solution, String fileName) {
        try (FileWriter writer = new FileWriter(fileName)) {
            writer.append("TripId,AccessStationId,EgressStationId,TotalTravelTime,TravelTimeChange,FlightDistanceChange,DepartureRedirectionRate,ArrivalRedirectionRate\n");
            for (int i = 0; i < solution.length; i++) {
                UAMTrip trip = subTrips.get(i);
                String tripId = trip.getTripId();
                double travelTimeChange = finalSolutionTravelTimeChanges.getOrDefault(tripId, 0.0);
                double flightDistanceChange = finalSolutionFlightDistanceChanges.getOrDefault(tripId, 0.0);
                double departureRedirectionRate = finalSolutionDepartureRedirectionRate.getOrDefault(tripId, 0.0);
                double arrivalRedirectionRate = finalSolutionArrivalRedirectionRate.getOrDefault(tripId, 0.0);
                String assignedAccessStation = finalSolutionAssignedAccessStation.getOrDefault(tripId, "N/A");
                String assignedEgressStation = finalSolutionAssignedEgressStation.getOrDefault(tripId, "N/A");
                double totalTravelTime = finalSolutionTotalTravelTime.getOrDefault(tripId, 0.0);

                writer.append(String.format("%s,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        tripId, assignedAccessStation, assignedEgressStation, totalTravelTime, travelTimeChange, flightDistanceChange, departureRedirectionRate, arrivalRedirectionRate));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    // Method to count the vehicles by capacity
    private static Map<Integer, Integer> countVehicleCapacities(int[] solution) {
        Map<Integer, Integer> vehicleLoadCount = new HashMap<>();
        for (int vehicleId : solution) {
            vehicleLoadCount.put(vehicleId, vehicleLoadCount.getOrDefault(vehicleId, 0) + 1);
        }

        Map<Integer, Integer> capacityCount = new HashMap<>();
        for (int load : vehicleLoadCount.values()) {
            capacityCount.put(load, capacityCount.getOrDefault(load, 0) + 1);
        }

        return capacityCount;
    }

    // Initial data extraction methods =================================================================================
    private static ArrayList<UAMTrip> extractSubTrips(List<UAMTrip> uamTrips) {
        // extract sub trips from uamTrips based on the departure time of trips falling between buffer start and end time
        return uamTrips.stream()
                .filter(trip -> trip.getDepartureTime() >= BUFFER_START_TIME && trip.getDepartureTime() < BUFFER_END_TIME)
                .collect(Collectors.toCollection(ArrayList::new));
    }

    // Method to create UAM vehicles and assign them to stations in the initialization phase (could also be used in later stage)
    private static void saveStationVehicleNumber(List<UAMTrip> subTrips) {

/*        // Loop through the stations so that we can assign at least 1 to each station before we assign more vehicles based on the demand
        for (UAMStation station : stations.values()) {
            UAMVehicle vehicle = createVehicle(station, vehicleTypes.get(vehicleTypeId));

            // Get the station ID
            Id<UAMStation> stationId = station.getId();
            // Check if there is already a list for this station ID, if not, create one
            List<UAMVehicle> vehiclesAtStation = originStationVehicleMap.computeIfAbsent(stationId, k -> new ArrayList<>());
            // Add the new vehicle to the list
            vehiclesAtStation.add(vehicle);
            vehicles.put(vehicle.getId(), vehicle);
            vehicleOriginStationMap.put(vehicle.getId(), station);
            vehicleDestinationStationMap.put(vehicle.getId(), ?);
            originStationVehicleMap.put(stationId, vehiclesAtStation);
        }*/

        // save the station's vehicle number for the current time based on the UAMTrips' origin station
        for (UAMTrip subTrip : subTrips) {
            feedDataForVehicleCreation(subTrip, true);
        }
    }
    private static UAMVehicle feedDataForVehicleCreation(UAMTrip subTrip, boolean isAddingVehicleBeforeInitialization) {
        UAMStation nearestOriginStation = findNearestStation(subTrip, stations, true);
        UAMStation nearestDestinationStation = findNearestStation(subTrip, stations, false);

        if (nearestOriginStation == null || nearestDestinationStation == null) {
            log.error("Found null station for trip: " + subTrip.getTripId());
        }

        UAMVehicle vehicle = createVehicle(nearestOriginStation);

        //vehicles.put(vehicle.getId(), vehicle);
        vehicleOriginStationMap.put(vehicle.getId(), nearestOriginStation);
        vehicleDestinationStationMap.put(vehicle.getId(), nearestDestinationStation);
        //vehicleOccupancyMap.put(vehicle, VEHICLE_CAPACITY);

        if (isAddingVehicleBeforeInitialization){
            // Get the station ID
            Id<UAMStation> nearestOriginStationId = nearestOriginStation.getId();
            // Check if there is already a list for this station ID, if not, create one
            List<UAMVehicle> vehiclesAtStation = originStationVehicleMap.computeIfAbsent(nearestOriginStationId, k -> new ArrayList<>());
            // Add the new vehicle to the list
            vehiclesAtStation.add(vehicle);
            originStationVehicleMap.put(nearestOriginStationId, vehiclesAtStation);
        }
        return vehicle;
    }
    // vehicle creator function
    private static UAMVehicle createVehicle(UAMStation uamStation) {
        /*        UAMVehicleType vehicleType = new UAMVehicleType(id, capacity, range, horizontalSpeed, verticalSpeed,
                boardingTime, deboardingTime, turnAroundTime, energyConsumptionVertical, energyConsumptionHorizontal,
                maximumCharge);*/
        //final Map<Id<UAMVehicleType>, UAMVehicleType> vehicleTypes = new HashMap<>();
        Id<UAMVehicleType> vehicleTypeId = Id.create("poolingVehicle", UAMVehicleType.class);
        UAMVehicleType vehicleType = new UAMVehicleType(vehicleTypeId, 0, 0, 0, 0,
                0, 0, 0);
        //vehicleTypes.put(vehicleTypeId, vehicleType);
        
        // Create a builder instance
        ImmutableDvrpVehicleSpecification.Builder builder = ImmutableDvrpVehicleSpecification.newBuilder();
        // Set the properties of the vehicle
        builder.id(Id.create(String.valueOf(FIRST_UAM_VEHICLE_ID++), DvrpVehicle.class));
        builder.startLinkId(uamStation.getLocationLink().getId());
        builder.capacity(VEHICLE_CAPACITY);
        builder.serviceBeginTime(BUFFER_START_TIME);
        builder.serviceEndTime(END_SERVICE_TIME_OF_THE_DAY);
        // Build the vehicle specification
        ImmutableDvrpVehicleSpecification vehicleSpecification = builder.build();

        return new UAMVehicle(vehicleSpecification,
                uamStation.getLocationLink(), uamStation.getId(), vehicleType);
    }
    private static UAMStation findNearestStation(UAMTrip trip, Map<Id<UAMStation>, UAMStation> stations, boolean accessLeg) {
        UAMStation nearestStation = null;
        double shortestDistance = Double.MAX_VALUE;
        for (UAMStation station : stations.values()) {
            if (station == null) {
                log.error("Encountered null station in stations map");
                continue; // Skip null stations
            }
            double distance = accessLeg ? trip.calculateAccessTeleportationDistance(station) : trip.calculateEgressTeleportationDistance(station);
            if (distance < shortestDistance) {
                nearestStation = station;
                shortestDistance = distance;
            }
        }
        if (nearestStation == null) {
            log.warn("No nearest station found for trip: " + trip.getTripId());
        }
        return nearestStation;
    }

    // read demand select randomly =====================================================================================
    public static List<UAMTrip> readTripsFromCsv(String filePath) {
        List<UAMTrip> trips = new ArrayList<>();

        try (Stream<String> lines = Files.lines(Paths.get(filePath))) {
            trips = lines
                    .skip(1) // Skip header line
                    .map(GeneticAlgorithm::parseTrip)
                    .collect(Collectors.toList());
        } catch (IOException e) {
            e.printStackTrace();
        }

        return trips;
    }
    private static UAMTrip parseTrip(String line) {
        String[] fields = line.split(",");
        String tripId = fields[0];
        double originX = Double.parseDouble(fields[1]);
        double originY = Double.parseDouble(fields[2]);
        double destX = Double.parseDouble(fields[3]);
        double destY = Double.parseDouble(fields[4]);
        double departureTime = Double.parseDouble(fields[5]);
        String purpose = fields[6];

        // Default values for optional fields
        double flightDistance = 0.0;
        UAMStation origStation = null;
        UAMStation destStation = null;
        String income = "0";

        return new UAMTrip(tripId, originX, originY, destX, destY, departureTime, flightDistance, origStation, destStation, purpose, income);
    }

    // Simulated Annealing to Repair Infeasible Solutions ==============================================================
    private static void repairInfeasibleSolutionsSA(int numProcessors, ExecutorService executorService, ArrayBlockingQueue<SolutionFitnessPair> queue) throws InterruptedException {

        // Add all infeasible solutions to the queue
        for (SolutionFitnessPair solutionPair : solutionsHeap) {
            if (!isFeasible(solutionPair.getSolution(), true)) {
                queue.add(solutionPair);
            } else {
                repairedSolutionsHeap.add(solutionPair); //TODO: could also be "simulated annealed" for better performance
            }
        }

        //executorService.invokeAll(tasks);
        // Execute tasks and wait for completion
        ThreadCounter threadCounter = new ThreadCounter();
        int counter = 0;
        int taskSize = queue.size();
        if (!queue.isEmpty()){
            log.info("Simulated Annealing starts...");
        }
        log.info("Que size is: " + taskSize);
        while (!queue.isEmpty()) {

            while (threadCounter.getProcesses() >= numProcessors - 1)
                Thread.sleep(200);

            SolutionFitnessPair solutionPair = queue.poll();
            if (solutionPair != null) {
                executorService.execute(new SimulatedAnnealing(solutionPair.getSolution(), threadCounter));
            }

            counter++;
            log.info("Calculation completion: " + counter + "/" + taskSize + " ("
                    + String.format("%.0f", (double) counter / taskSize * 100) + "%).");
        }
        executorService.shutdown();
    }

    // Simulated Annealing Method
    static class SimulatedAnnealing implements Runnable {
        private int[] solution;
        private ThreadCounter threadCounter;
        SimulatedAnnealing(int[] solution, ThreadCounter threadCounter){
            this.solution = solution;
            this.threadCounter = threadCounter;
        }

        @Override
        public void run() {
            threadCounter.register();

            double temperature = 100000.0;
            double coolingRate = 0.003;

            int[] currentSolution = Arrays.copyOf(solution, solution.length);
            int[] bestSolution = Arrays.copyOf(currentSolution, currentSolution.length);
            double bestFitness = calculateFitness(bestSolution, false);

            while (temperature > 1) {
                int[] newSolution = Arrays.copyOf(currentSolution, currentSolution.length);

                // Randomly change vehicle assignment to repair the solution
                int tripIndex = rand.nextInt(newSolution.length);
                assignAvailableVehicle(tripIndex, newSolution);

                double currentFitness = calculateFitness(currentSolution, false);
                double newFitness = calculateFitness(newSolution, false);

                if (acceptanceProbability(currentFitness, newFitness, temperature) > rand.nextDouble()) {
                    currentSolution = Arrays.copyOf(newSolution, newSolution.length);
                }

                if (newFitness > bestFitness && isFeasible(newSolution, false)) {
                    bestSolution = Arrays.copyOf(newSolution, newSolution.length);
                    bestFitness = newFitness;
                }

                temperature *= 1 - coolingRate;
            }

            if (isFeasible(bestSolution, false)) {
                repairedSolutionsHeap.add(new SolutionFitnessPair(bestSolution, bestFitness));
            }

            threadCounter.deregister();
        }
        // Acceptance Probability Calculation for Simulated Annealing
        private static double acceptanceProbability(double currentFitness, double newFitness, double temperature) {
            if (newFitness > currentFitness) {
                return 1.0;
            }
            return Math.exp((newFitness - currentFitness) / temperature);
        }
    }
    public static class ThreadCounter {
        private int processes;

        public synchronized void register() {
            processes++;
        }

        public synchronized void deregister() {
            processes--;
        }

        public synchronized int getProcesses() {
            return processes;
        }
    }

    // Google OR tools (version: 9.10.4067) ============================================================================
    static {
        // Load the OR-Tools native library
        Loader.loadNativeLibraries();
    }
    private static void repairInfeasibleSolutions(int numProcessors, ExecutorService executorService, ArrayBlockingQueue<SolutionFitnessPair> queue) throws InterruptedException {
        // Add all infeasible solutions to the queue
        for (SolutionFitnessPair solutionPair : solutionsHeap) {
            if (!isFeasible(solutionPair.getSolution(), true)) {
                queue.add(solutionPair);
            } else {
                repairedSolutionsHeap.add(solutionPair); // TODO: could also be "simulated annealed" for better performance
            }
        }

        ThreadCounter threadCounter = new ThreadCounter();
        int counter = 0;
        int taskSize = queue.size();
        if (!queue.isEmpty()) {
            log.info("Constraint Programming (CP) starts...");
        }
        log.info("Queue size is: " + taskSize);
        while (!queue.isEmpty()) {
            while (threadCounter.getProcesses() >= numProcessors/bufferDivider - 1)
                Thread.sleep(200);

            SolutionFitnessPair solutionPair = queue.poll();
            if (solutionPair != null) {
                executorService.execute(new Runnable() {
                    @Override
                    public void run() {
                        threadCounter.register();
                        try {
                            int[] solution = solutionPair.getSolution();
                            if (fixInfeasibleSolutionWithORTools(solution)) {
                                repairedSolutionsHeap.add(new SolutionFitnessPair(solution, calculateFitness(solution, false)));
                            }
                        } finally {
                            threadCounter.deregister();
                        }
                    }
                });
            }

            counter++;
            log.info("Calculation completion: " + counter + "/" + taskSize + " (" + String.format("%.0f", (double) counter / taskSize * 100) + "%).");
        }
        executorService.shutdown();
    }
    private static boolean fixInfeasibleSolutionWithORTools(int[] solution) {
        Solver solver = null;
        try {
            solver = new Solver("FixInfeasibleSolution");

            int numTrips = solution.length;

            // Variables: Vehicle assignment for each trip
            IntVar[] vehicleAssignments = new IntVar[numTrips];
            for (int i = 0; i < numTrips; i++) {
                UAMTrip trip = subTrips.get(i);
                List<UAMVehicle> availableVehicles = tripVehicleMap.get(trip.getTripId());
                int[] availableVehicleIds = availableVehicles.stream().mapToInt(v -> Integer.parseInt(v.getId().toString())).toArray();

                // Define the domain of the variable to be the available vehicles for this trip
                vehicleAssignments[i] = solver.makeIntVar(availableVehicleIds, "vehicle_" + i);
            }

            // Constraints: Each vehicle cannot exceed its capacity
            for (int vehicleId : tripVehicleMap.values().stream().flatMap(List::stream).mapToInt(v -> Integer.parseInt(v.getId().toString())).distinct().toArray()) {
                IntVar vehicleCapacityUsage = solver.makeIntVar(0, VEHICLE_CAPACITY, "vehicleCapacityUsage_" + vehicleId);
                IntVar[] vehicleLoad = new IntVar[numTrips];
                for (int i = 0; i < numTrips; i++) {
                    vehicleLoad[i] = solver.makeIsEqualCstVar(vehicleAssignments[i], vehicleId);
                }
                solver.addConstraint(solver.makeEquality(vehicleCapacityUsage, solver.makeSum(vehicleLoad)));
                solver.addConstraint(solver.makeLessOrEqual(vehicleCapacityUsage, VEHICLE_CAPACITY));
            }

            // Objective: Minimize the number of changes in vehicle assignments
            int[] initialAssignments = Arrays.copyOf(solution, solution.length);
            IntVar[] assignmentChanges = new IntVar[numTrips];
            for (int i = 0; i < numTrips; i++) {
                assignmentChanges[i] = solver.makeIsDifferentCstVar(vehicleAssignments[i], initialAssignments[i]);
            }
            IntVar totalChanges = solver.makeSum(assignmentChanges).var();
            OptimizeVar objective = solver.makeMinimize(totalChanges, 1);

            // Set up the search parameters with number of workers (threads)
            // There is no direct way to set number of threads in the CP solver in Java as in other APIs.
            // But OR-Tools generally respects the system's thread settings, you can try using environment variables.

            // Solve the problem
            DecisionBuilder db = solver.makePhase(vehicleAssignments, Solver.CHOOSE_FIRST_UNBOUND, Solver.ASSIGN_MIN_VALUE);
            solver.newSearch(db, objective);

            boolean feasibleSolutionFound = false;
            while (solver.nextSolution()) {
/*                boolean allConstraintsSatisfied = true;
                for (int vehicleId : tripVehicleMap.values().stream().flatMap(List::stream).mapToInt(v -> Integer.parseInt(v.getId().toString())).distinct().toArray()) {
                    int vehicleUsage = 0;
                    for (int i = 0; i < numTrips; i++) {
                        if (vehicleAssignments[i].value() == vehicleId) {
                            vehicleUsage++;
                        }
                    }
                    if (vehicleUsage > VEHICLE_CAPACITY) {
                        allConstraintsSatisfied = false;
                        break;
                    }
                }*/
                //if (allConstraintsSatisfied) {
                    for (int i = 0; i < numTrips; i++) {
                        solution[i] = (int) vehicleAssignments[i].value();
                    }
                    feasibleSolutionFound = true;
                    break;
                //}
            }
            solver.endSearch();
            return feasibleSolutionFound;
        } finally {
            if (solver != null) {
                solver = null;  // Ensure the solver is closed properly
            }
            // Suggest the JVM to run garbage collection
            System.gc();
            System.runFinalization();
        }
    }

}

//TODO: only calculate the fitness if assignment for the trip is changed
//TODO: To sort trips based on their origin station and destination station?
//TODO: 3.	Heuristic-Based Mutation: Implement domain-specific mutations such as swapping vehicle assignments between closely located trips.
//TODO: Need use the best solution starting from the crossover_disable_after iteration to generate new solutions for remaining iterations