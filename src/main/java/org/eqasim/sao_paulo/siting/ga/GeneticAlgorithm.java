package org.eqasim.sao_paulo.siting.ga;

import net.bhl.matsim.uam.infrastructure.UAMStation;
import org.eqasim.sao_paulo.siting.Utils.*;

import org.apache.log4j.Logger;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.*;
import java.util.stream.Collectors;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.io.FileWriter;
import java.util.stream.Stream;

import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import net.bhl.matsim.uam.infrastructure.UAMVehicleType;
import org.matsim.contrib.dvrp.fleet.ImmutableDvrpVehicleSpecification;
import org.matsim.api.core.v01.Id;

public class GeneticAlgorithm {
    private static final Logger log = Logger.getLogger(GeneticAlgorithm.class);

    // Genetic Algorithm parameters ====================================================================================
    private static final int MAX_GENERATIONS = 100; // Max number of generations
    private static final int CROSSOVER_DISABLE_AFTER = 100; // New field to control when to stop crossover
    private static final int POP_SIZE = 100; // Population size
    private static final double MUTATION_RATE = 0.05; // Mutation rate
    private static final double CROSSOVER_RATE = 0.7; // Crossover rate
    private static final int TOURNAMENT_SIZE = 5; // Tournament size for selection

    private static final double ALPHA = - 10.0; // Weight for changed flight distances
    private static final double BETA = - 0.1; // Weight for change in travel time
    private static final double BETA_NONE_POOLED_TRIP_EARLIER_DEPARTURE = - 0.1; //TODO: need to reconsider the value
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
    private static final double SEARCH_RADIUS_ORIGIN = 4000; // search radius for origin station
    private static final double SEARCH_RADIUS_DESTINATION = 4000; // search radius for destination station

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
    //private static final Map<Id<DvrpVehicle>, UAMVehicle> vehicles = new HashMap<>();
    private static final Map<Id<UAMStation>, List<UAMVehicle>> originStationVehicleMap = new HashMap<>();
    private static final Map<Id<DvrpVehicle>, UAMStation> vehicleOriginStationMap = new HashMap<>();
    private static final Map<Id<DvrpVehicle>, UAMStation> vehicleDestinationStationMap = new HashMap<>();
    private static Map<String, Map<UAMVehicle, Integer>> tripVehicleMap = null;

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

    // Main method to run the GA =======================================================================================
    public static void main(String[] args) throws IOException, InterruptedException {
        // Load data
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();

        subTrips = extractSubTrips(dataLoader.getUamTrips());
        /*        String filePath = "/home/tumtse/Documents/haowu/uam/uam/scenarios/1-percent/sao_paulo_population2trips.csv";
        subTrips = readTripsFromCsv(filePath);
        //Randomly select 10% trips from the list of subTrips
        subTrips = subTrips.stream()
                .filter(trip -> rand.nextDouble() < 0.1)
                .collect(Collectors.toCollection(ArrayList::new));*/

        //vehicles = dataLoader.getVehicles();
        stations = dataLoader.getStations();

        // Initialize the origin station and destination station for each trip
        for(UAMTrip uamTrip: subTrips){
            uamTrip.setOriginStation(findNearestStation(uamTrip, stations, true));
            uamTrip.setDestinationStation(findNearestStation(uamTrip, stations, false));
        }
        saveStationVehicleNumber(subTrips);
        tripVehicleMap = findNearbyVehiclesToTrips(subTrips);

        // Initialize population and solutions heap in the first generation
        int[][] population = initializePopulation();
        if (solutionsHeap.isEmpty()) {
            for (int[] individual : population) {
                double fitness = calculateFitness(individual, false);
                solutionsHeap.add(new SolutionFitnessPair(individual, fitness));
            }
            System.out.println("Generation " + "0" + ": Best fitness = " + findBestFitness(population));
        }

        // GA iterations
        for (int gen = 1; gen < MAX_GENERATIONS; gen++) {
            population = evolvePopulation(population, gen);
            updateSolutionsHeap(population);
            System.out.println("Generation " + gen + ": Best fitness = " + findBestFitness(population));
        }

        // Repair infeasible solutions using Simulated Annealing
        // Executor service and queue for parallel processing
        final int numProcessors = Runtime.getRuntime().availableProcessors();
        ExecutorService executorService = Executors.newFixedThreadPool(numProcessors);
        ArrayBlockingQueue<SolutionFitnessPair> queue = new ArrayBlockingQueue<>(solutionsHeap.size());
        repairInfeasibleSolutions(numProcessors, executorService, queue);
        // Make sure that the file is not written before all threads are finished
        while (!executorService.isTerminated())
            Thread.sleep(200);

        // Find the best feasible solution at the end of GA execution without altering the original solutions heap
        SolutionFitnessPair bestFeasibleSolutionFitnessPair = findBestFeasibleSolution();
        int[] bestFeasibleSolution = bestFeasibleSolutionFitnessPair.getSolution();
        System.out.println("Best feasible solution: " + Arrays.toString(bestFeasibleSolution));
        System.out.println("The fitness of the best feasible solution: " + bestFeasibleSolutionFitnessPair.getFitness());

        // Calculate and print the performance indicators
        calculateFitness(bestFeasibleSolution, true);
        printPerformanceIndicators(bestFeasibleSolution, "src/main/java/org/eqasim/sao_paulo/siting/ga/trip_statistics.csv");

        // Print the NUMBER_OF_TRIPS_LONGER_THAN
        System.out.println("Threshold for trips longer than " + THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING + ": " + NUMBER_OF_TRIPS_LONGER_TAHN);
    }

    // GA ==============================================================================================================
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
        for (int i = 0; i < individual.length; i++) {
            assignAvailableVehicle(i, individual);
        }
        resetVehicleCapacities(tripVehicleMap); // Reset the vehicle capacity since capacity of vehicles will be updated during each individual generation
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
        for (int i = 0; i < individual.length; i++) {
            if (rand.nextDouble() < MUTATION_RATE) {
                assignAvailableVehicle(i, individual);
            }
        }
        resetVehicleCapacities(tripVehicleMap); // Reset the vehicle capacity since capacity of vehicles will be updated during each individual generation
        return individual;
    }

    private static void assignAvailableVehicle(int i, int[] individual) {
        UAMTrip trip = subTrips.get(i);
        Map<UAMVehicle, Integer> vehicleCapacityMap = tripVehicleMap.get(trip.getTripId());
        //handle the case when there is no available vehicle
        if (vehicleCapacityMap == null) {
            throw new IllegalArgumentException("No available vehicle for the trip, Please increase the search radius.");
        }
        List<UAMVehicle> vehicleList = vehicleCapacityMap.entrySet().stream()
                .filter(entry -> entry.getValue() > 0)
                .map(Map.Entry::getKey)
                .collect(Collectors.toCollection(ArrayList::new));

        if (!vehicleList.isEmpty()) {
            //add egress constraint
            Iterator<UAMVehicle> iterator = vehicleList.iterator();
            while (iterator.hasNext()) {
                UAMVehicle vehicle = iterator.next();
                UAMStation destinationStation = vehicleDestinationStationMap.get(vehicle.getId());
                if (trip.calculateEgressTeleportationDistance(destinationStation) > SEARCH_RADIUS_DESTINATION) {
                    iterator.remove();
                }
            }

            // ----- add a new vehicle for the trip when there is no available vehicle (i.e., vehicle still has capacity) after checking the egress constraint
            if (vehicleList.isEmpty()) {
                UAMVehicle vehicle = feedDataForVehicleCreation(trip, false);
                vehicleCapacityMap.put(vehicle, VEHICLE_CAPACITY);
                tripVehicleMap.put(trip.getTripId(), vehicleCapacityMap);
                vehicleList.add(vehicle);
            }

            //add access constraint
            int vehicleIndex = rand.nextInt(vehicleList.size());
            UAMVehicle selectedVehicle = vehicleList.get(vehicleIndex);
            Integer currentCapacity = vehicleCapacityMap.get(selectedVehicle);
            if (currentCapacity > 0) {
                individual[i] = Integer.parseInt(selectedVehicle.getId().toString());

                // Decrement capacity and explicitly update tripVehicleMap
                vehicleCapacityMap.put(selectedVehicle, currentCapacity - 1);
                tripVehicleMap.put(trip.getTripId(), vehicleCapacityMap);
            } else {
                if (currentCapacity < 0){
                    throw new IllegalArgumentException("Capacity of the selected vehicle is smaller than 1.");
                }
            }
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

            // Find the base trip (the trip with the earliest arrival time at the departure UAM station)
            UAMTrip baseTrip = trips.get(0);
            for (UAMTrip trip : trips) {
                double accessTimeOfBaseTrip = baseTrip.calculateAccessTeleportationTime(originStationOfVehicle);
                double accessTimeOfPooledTrip = trip.calculateAccessTeleportationTime(originStationOfVehicle);
                if ((trip.getDepartureTime() + accessTimeOfPooledTrip) >= (baseTrip.getDepartureTime() + accessTimeOfBaseTrip)) {
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
                tripFlightDistanceChange += flightDistanceChange;
                if(isFinalBestFeasibleSolution){
                    finalSolutionFlightDistanceChanges.put(trip.getTripId(), tripFlightDistanceChange);
                }
                // calculate change in flight time due to the change in flight distance
                double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
                fitness += BETA * flightTimeChange;
                tripTimeChange += flightTimeChange;
                // calculate additional travel time
                double originalArrivalTimeForThePooledTrip = trip.getDepartureTime() + trip.calculateAccessTeleportationTime(trip.getOriginStation());
                double additionalTravelTimeDueToAccessMatching = boardingTimeForAllTrips - originalArrivalTimeForThePooledTrip;
                if (additionalTravelTimeDueToAccessMatching < 0){
                    additionalTravelTimeDueToAccessMatching = 0;
                    //TODO: reconsider for "negative additional travel time" cases
                }
                fitness += BETA * additionalTravelTimeDueToAccessMatching;
                tripTimeChange += additionalTravelTimeDueToAccessMatching;
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
        if(travelTimeChangeDueToAccessMatching > 0) {
            fitness += BETA * travelTimeChangeDueToAccessMatching;
            tripTimeChange += travelTimeChangeDueToAccessMatching;
        } else {
            //fitness += BETA_NONE_POOLED_TRIP_EARLIER_DEPARTURE * travelTimeChangeDueToAccessMatching;
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
    private static void resetVehicleCapacities(Map<String, Map<UAMVehicle, Integer>> tripVehicleMap) {
        for (Map<UAMVehicle, Integer> vehicleMap : tripVehicleMap.values()) {
            vehicleMap.forEach((vehicle, capacity) -> {
                vehicleMap.put(vehicle, VEHICLE_CAPACITY); // Reset capacity to 4
            });
        }
    }
    private static Map<String, Map<UAMVehicle, Integer>> findNearbyVehiclesToTrips(List<UAMTrip> subTrips) {
        Map<String, Map<UAMVehicle, Integer>> tripVehicleMap = new HashMap<>();
        for (UAMTrip trip : subTrips) {
            for (UAMStation station : stations.values()) {
                if (trip.calculateAccessTeleportationDistance(station) <= SEARCH_RADIUS_ORIGIN) {
                    List<UAMVehicle> vehicles = originStationVehicleMap.get(station.getId());
                    if (vehicles == null){
                        continue;
                    }
                    Map<UAMVehicle, Integer> existingVehicles = tripVehicleMap.getOrDefault(trip.getTripId(), new HashMap<>());

                    for (UAMVehicle vehicle : vehicles) {
                        existingVehicles.put(vehicle, VEHICLE_CAPACITY);
                    }

                    tripVehicleMap.put(trip.getTripId(), existingVehicles);
                } else {
                    if (trip.calculateAccessTeleportationDistance(station)> THRESHOLD_FOR_TRIPS_LONGER_THAN){
                        NUMBER_OF_TRIPS_LONGER_TAHN++;
                    }

                    // ----- Add a new vehicle for the trip when the access teleportation distance is longer than the search radius
                    Map<UAMVehicle, Integer> vehicleCapacityMap = tripVehicleMap.getOrDefault(trip.getTripId(), new HashMap<>());
                    UAMVehicle vehicle = feedDataForVehicleCreation(trip, false);
                    vehicleCapacityMap.put(vehicle, VEHICLE_CAPACITY);
                    tripVehicleMap.put(trip.getTripId(), vehicleCapacityMap);
                }
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
    // New method to update the solutions heap
    private static void updateSolutionsHeap(int[][] population) {
        for (int[] individual : population) {
            double fitness = calculateFitness(individual, false);
            // Only consider adding if the new solution is better than the worst in the heap
            if (fitness > solutionsHeap.peek().getFitness()) {
                if (solutionsHeap.size() == POP_SIZE) {
                    solutionsHeap.poll(); // Remove the solution with the lowest fitness
                    solutionsHeap.add(new SolutionFitnessPair(individual, fitness)); // Add the new better solution
                } else {
                    throw new IllegalArgumentException("Need to handle the case when the heap size exceeds the population size.");
                }
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
            if (isFeasible(candidateSolution)) {
                return solutionPair;
            }
        }
        throw new IllegalStateException("No feasible solution found in the entire population");
    }
    // Helper method to check if a solution violates vehicle capacity constraints
    private static boolean isFeasible(int[] solution) {
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
        //System.out.println("Number of vehicles with capacity violation: " + vehicleCapacityViolated);
        return isFeasible;
    }

    // Performance indicators ==========================================================================================
    // Find the best fitness in the current population
    private static double findBestFitness(int[][] population) {
        double bestFitness = -Double.MAX_VALUE;
        for (int[] individual : population) {
            double fitness = calculateFitness(individual, false);
            if (fitness > bestFitness) {
                bestFitness = fitness;
            }
        }
        return bestFitness;
    }
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

        Collection<Double> travelTimeChanges = finalSolutionTravelTimeChanges.values();
        List<Double> sortedTravelTimeChanges = new ArrayList<>(travelTimeChanges);
        Collections.sort(sortedTravelTimeChanges);

        double averageTravelTime = sortedTravelTimeChanges.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);
        double percentile5thTravelTime = sortedTravelTimeChanges.get((int) (0.05 * sortedTravelTimeChanges.size()));
        double percentile95thTravelTime = sortedTravelTimeChanges.get((int) (0.95 * sortedTravelTimeChanges.size()) - 1);

        System.out.println("Average travel time change: " + averageTravelTime);
        System.out.println("5th percentile of travel time change: " + percentile5thTravelTime);
        System.out.println("95th percentile of travel time change: " + percentile95thTravelTime);

        Collection<Double> flightDistanceChanges = finalSolutionFlightDistanceChanges.values();
        List<Double> sortedFlightDistanceChanges = new ArrayList<>(flightDistanceChanges);
        Collections.sort(sortedFlightDistanceChanges);

        double averageFlightDistance = sortedFlightDistanceChanges.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);
        double percentile5thFlightDistance = sortedFlightDistanceChanges.get((int) (0.05 * sortedFlightDistanceChanges.size()));
        double percentile95thFlightDistance = sortedFlightDistanceChanges.get((int) (0.95 * sortedFlightDistanceChanges.size()) - 1);

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
        double percentile5thDepartureRedirectionRate = sortedDepartureRedirectionRates.get((int) (0.05 * sortedDepartureRedirectionRates.size()));
        double percentile95thDepartureRedirectionRate = sortedDepartureRedirectionRates.get((int) (0.95 * sortedDepartureRedirectionRates.size()) - 1);

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
        double percentile5thArrivalRedirectionRate = sortedArrivalRedirectionRates.get((int) (0.05 * sortedArrivalRedirectionRates.size()));
        double percentile95thArrivalRedirectionRate = sortedArrivalRedirectionRates.get((int) (0.95 * sortedArrivalRedirectionRates.size()) - 1);

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
        UAMVehicle vehicle = createVehicle(nearestOriginStation);

        //vehicles.put(vehicle.getId(), vehicle);
        vehicleOriginStationMap.put(vehicle.getId(), nearestOriginStation);
        vehicleDestinationStationMap.put(vehicle.getId(), nearestDestinationStation);

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
        for (UAMStation station: stations.values()){
            double distance;
            if (accessLeg){
                distance = trip.calculateAccessTeleportationDistance(station);
            } else {
                distance = trip.calculateEgressTeleportationDistance(station);
            }
            if (distance < shortestDistance){
                nearestStation = station;
                shortestDistance = distance;
            }
        }
        return nearestStation;
    }

    // read demand =====================================================================================================
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

    // Simulated Annealing to Repair Infeasible Solutions =============================================================
    private static void repairInfeasibleSolutions(int numProcessors, ExecutorService executorService, ArrayBlockingQueue<SolutionFitnessPair> queue) throws InterruptedException {

        // Add all infeasible solutions to the queue
        for (SolutionFitnessPair solutionPair : solutionsHeap) {
            if (!isFeasible(solutionPair.getSolution())) {
                queue.add(solutionPair);
            } else {
                repairedSolutionsHeap.add(solutionPair); //TODO: could also be "simulated annealed" for better performance
            }
        }

        //executorService.invokeAll(tasks);
        // Execute tasks and wait for completion
        ThreadCounter threadCounter = new ThreadCounter();
        log.info("Simulated Annealing starts...");
        int counter = 0;
        int taskSize = queue.size();
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

            double temperature = 1000.0;
            double coolingRate = 0.003;

            int[] currentSolution = Arrays.copyOf(solution, solution.length);
            int[] bestSolution = Arrays.copyOf(currentSolution, currentSolution.length);
            double bestFitness = calculateFitness(bestSolution, false);

            while (temperature > 1) {
                int[] newSolution = Arrays.copyOf(currentSolution, currentSolution.length);

                // Randomly change vehicle assignment to repair the solution
                int tripIndex = rand.nextInt(newSolution.length);
                assignAvailableVehicle(tripIndex, newSolution);
                resetVehicleCapacities(tripVehicleMap); // Reset vehicle capacities

                double currentFitness = calculateFitness(currentSolution, false);
                double newFitness = calculateFitness(newSolution, false);

                if (acceptanceProbability(currentFitness, newFitness, temperature) > rand.nextDouble()) {
                    currentSolution = Arrays.copyOf(newSolution, newSolution.length);
                }

                if (newFitness > bestFitness && isFeasible(newSolution)) {
                    bestSolution = Arrays.copyOf(newSolution, newSolution.length);
                    bestFitness = newFitness;
                }

                temperature *= 1 - coolingRate;
            }

            double repairedFitness = calculateFitness(bestSolution, false);
            if (isFeasible(bestSolution)) {
                repairedSolutionsHeap.add(new SolutionFitnessPair(bestSolution, repairedFitness));
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

}
//TODO: To sort trips based on their origin station and destination station?
//TODO: 3.	Heuristic-Based Mutation: Implement domain-specific mutations such as swapping vehicle assignments between closely located trips.
//TODO: Need use the best solution starting from the crossover_disable_after iteration to generate new solutions for remaining iterations