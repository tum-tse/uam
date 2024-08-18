package org.eqasim.sao_paulo.siting.ga;

import net.bhl.matsim.uam.infrastructure.readers.UAMXMLReader;
import org.eqasim.sao_paulo.siting.Utils.*;

import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Network;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import org.matsim.contrib.dvrp.fleet.ImmutableDvrpVehicleSpecification;
import net.bhl.matsim.uam.infrastructure.UAMStation;
import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import net.bhl.matsim.uam.infrastructure.UAMVehicleType;

import org.apache.log4j.Logger;
import com.google.ortools.Loader;
import com.google.ortools.constraintsolver.*;
import com.google.ortools.constraintsolver.IntVar;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.network.io.MatsimNetworkReader;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.*;
import java.util.stream.Collectors;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.io.FileWriter;
import java.util.stream.Stream;
import java.util.concurrent.ConcurrentHashMap;

public class MultiObjectiveNSGAII {
    private static final Logger log = Logger.getLogger(MultiObjectiveNSGAII.class);

    // Genetic Algorithm parameters ====================================================================================
    private static final int MAX_GENERATIONS = 100; // Max number of generations
    private static final int CROSSOVER_DISABLE_AFTER = 100; // New field to control when to stop crossover
    private static final int POP_SIZE = 50; // Population size
    private static final double MUTATION_RATE = 0.05; // Mutation rate
    private static final double CROSSOVER_RATE = 0.7; // Crossover rate
    private static final int TOURNAMENT_SIZE = 5; // Tournament size for selection
    private static boolean ENABLE_LOCAL_SEARCH = true; // Enable local search after each generation

    private static final double ALPHA = - 2.02 * 0.9101 / 1000; // Weight for changed flight distances
    private static final double BETA = - 64.0 / 3600; // Weight for change in travel time
    private static final double BETA_CRUCIAL_TIME_ChANGE = - 0.1; //TODO: need to reconsider the value
    private static final double PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION = -10000;

    private final long SEED = 4711; // MATSim default Random Seed
    private final Random rand = new Random(SEED);

    // Parameters and constant for the UAM problem =====================================================================
    private static int FIRST_UAM_VEHICLE_ID = 1;
    private static final int VALUE_FOR_NO_VEHICLE_AVAILABLE = -1; // For example, using -1 as an indicator of no vehicle available for a trip
    private static final double END_SERVICE_TIME_OF_THE_DAY = 3600*36; // End service time of the day
    private static final double VEHICLE_CRUISE_SPEED = 350000.0 / 3600.0; // Vehicle cruise speed in m/s
    private static final int VEHICLE_CAPACITY = 4; // Vehicle capacity

    // Variables for the UAM problem ===================================================================================
    private static double BUFFER_START_TIME = 3600*7; // Buffer start time for the first trip
    private static double BUFFER_END_TIME = 3600*7+600; // Buffer end time for the last trip
    private static double SEARCH_RADIUS_ORIGIN = 1500; // search radius for origin station
    private static double SEARCH_RADIUS_DESTINATION = 1500; // search radius for destination station

    // Helpers for the UAM problem =====================================================================================
    private static final double THRESHOLD_FOR_TRIPS_LONGER_THAN = SEARCH_RADIUS_ORIGIN;
    private static final String THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING = String.valueOf(THRESHOLD_FOR_TRIPS_LONGER_THAN);
    private static final int SHARED_RIDE_TRAVEL_TIME_CHANGE_THRESHOLD = 700;

    // Data container for the UAM problem ==============================================================================
    private static List<UAMTrip> trips;
    private List<UAMTrip> subTrips = null;
    private static Map<Id<UAMStation>, UAMStation> stations;
    private final Map<Id<UAMStation>, List<UAMVehicle>> originStationVehicleMap = new HashMap<>();
    private final Map<Id<DvrpVehicle>, UAMStation> vehicleOriginStationMap = new ConcurrentHashMap<>();
    private final Map<Id<DvrpVehicle>, UAMStation> vehicleDestinationStationMap = new ConcurrentHashMap<>();
    private Map<String, List<UAMVehicle>> tripVehicleMap = new ConcurrentHashMap<>(); // Update tripVehicleMap to use ConcurrentHashMap
    //private static final Map<UAMVehicle, Integer> vehicleOccupancyMap = new HashMap<>();

    // Data container for outputs
    private final PriorityQueue<SolutionFitnessPair> solutionsHeap = new PriorityQueue<>(Comparator.comparingDouble(p -> p.getFitness()[0])); // Modify comparator to use first fitness objective
    private final PriorityQueue<SolutionFitnessPair> repairedSolutionsHeap = new PriorityQueue<>(Comparator.comparingDouble(p -> p.getFitness()[0])); // Modify comparator to use first fitness objective
    private final PriorityQueue<SolutionFitnessPair> bestSolutionsAcrossGenerations = new PriorityQueue<>(
            (a, b) -> Double.compare(a.getFitness()[0], b.getFitness()[0])  // Min heap
    );
    private static final int MAX_BEST_SOLUTIONS = 100;  // Adjust as needed
    //private final Map<String, Double> finalSolutionTravelTimeChanges = new HashMap<>(); // Additional field to store travel time change of each trip for the final best feasible solution
    //private final Map<String, Double> finalSolutionFlightDistanceChanges = new HashMap<>(); // Additional field to store saved flight distance of each trip for the final best feasible solution
    //private final Map<String, Double> finalSolutionDepartureRedirectionRate = new HashMap<>(); // Additional field to store redirection rate of each trip for the final best feasible solution
    //private final Map<String, Double> finalSolutionArrivalRedirectionRate = new HashMap<>(); // Additional field to store redirection rate of each trip for the final best feasible solution
    //private final Map<String, Double> finalSolutionTotalTravelTime = new HashMap<>(); // Additional field to store total travel time of each trip for the final best feasible solution
    //private final Map<String, String> finalSolutionAssignedAccessStation = new HashMap<>(); // Additional field to store assigned access station of each trip for the final best feasible solution
    //private final Map<String, String> finalSolutionAssignedEgressStation = new HashMap<>(); // Additional field to store assigned egress station of each trip for the final best feasible solution

    // Parallel computing
    private static final int numProcessors = Runtime.getRuntime().availableProcessors();
    private static final int bufferDivider = 1;
    private static String uamScenarioInputPath = "scenarios/1-percent/uam-scenario_400";
    private String outputFile = "src/main/java/org/eqasim/sao_paulo/siting/ga/results/vertiports_400";
    // TODO: Create an initial population of solutions using domain-specific knowledge (in our case is the vehicles which were used to create the initial fleet of the vehicles).
    // TODO: How to handle the extremely large travel time?

    // Static initializer block
    static {
        initializeData();
    }

    // Static method to initialize data
    private static void initializeData() {
        // Load data
/*        {
            DataLoader dataLoader = new DataLoader();
            dataLoader.loadAllData();
            //vehicles = dataLoader.getVehicles();
            stations = dataLoader.getStations();
        }*/
        Network network = NetworkUtils.createNetwork();
        new MatsimNetworkReader(network).readFile(uamScenarioInputPath + "/uam_network.xml.gz");
        UAMXMLReader uamReader = new UAMXMLReader(network);
        uamReader.readFile(uamScenarioInputPath + "/uam_vehicles.xml.gz");
        stations = uamReader.getStations();

        //subTrips = extractSubTrips(dataLoader.getUamTrips());
        String filePath = "scenarios/1-percent/sao_paulo_population2trips.csv";
        trips = readTripsFromCsv(filePath);
    }

    // Constructor
    public MultiObjectiveNSGAII() {
        // The constructor is now empty as initialization is done in the static block
    }

    // Main method for testing
    public static void main(String[] args) {
        MultiObjectiveNSGAII instance = new MultiObjectiveNSGAII();
        instance.runAlgorithm();
    }

    // Main method to run the the specifyed algorithm ==================================================================
    public static double[] callAlgorithm(String[] args) throws IOException, InterruptedException {
        if (args.length < 4) {
            System.out.println("Usage: java MultiObjectiveNSGAII <BUFFER_END_TIME> <SEARCH_RADIUS_ORIGIN> <SEARCH_RADIUS_DESTINATION> <ENABLE_LOCAL_SEARCH>");
            System.exit(1);
        }

        BUFFER_END_TIME = BUFFER_START_TIME + Double.parseDouble(args[0])*60;
        SEARCH_RADIUS_ORIGIN = Double.parseDouble(args[1]);
        SEARCH_RADIUS_DESTINATION = Double.parseDouble(args[2]);
        ENABLE_LOCAL_SEARCH = Boolean.parseBoolean(args[3]);

        MultiObjectiveNSGAII instance = new MultiObjectiveNSGAII();
        return instance.runAlgorithm();
    }
    public double[] runAlgorithm() {
        // Randomly select 10% trips from the list of subTrips
        subTrips = trips.stream()
                .filter(trip -> trip.getDepartureTime() >= BUFFER_START_TIME && trip.getDepartureTime() < BUFFER_END_TIME) // Add the filter
                .filter(trip -> rand.nextDouble() <= 1)
                .collect(Collectors.toCollection(ArrayList::new));

        log.info("The number of UAM trips: " + subTrips.size());

        // Initialize the origin station and destination station for each trip
        for (UAMTrip uamTrip : subTrips) {
            uamTrip.setOriginStation(findNearestStation(uamTrip, stations, true));
            uamTrip.setDestinationStation(findNearestStation(uamTrip, stations, false));
        }
        saveStationVehicleNumber(subTrips);
        tripVehicleMap = findNearbyVehiclesToTrips(subTrips);

        List<SolutionFitnessPair> population = initializePopulation();
        for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
            population = evolvePopulation(population, gen);
            // Find the best solution in the current population
            SolutionFitnessPair bestSolution = Collections.max(population, Comparator.comparingDouble(p -> p.getFitness()[0]));
            System.out.println("Generation " + gen + ": Best fitness = " + Arrays.toString(bestSolution.getFitness()));
            if (gen == MAX_GENERATIONS - 1) {
                calculatePopulationIndicators(population);
            }
        }

        // Find the best feasible solution at the end of GA execution without altering the original solutions heap
        SolutionFitnessPair bestFeasibleSolutionFitnessPair = findBestFeasibleSolution(population);
        int[] bestFeasibleSolution = bestFeasibleSolutionFitnessPair.getSolution();
        double [] bestFeasibleSolutionFitness = bestFeasibleSolutionFitnessPair.getFitness();
        System.out.println("Best feasible solution: " + Arrays.toString(bestFeasibleSolution));
        System.out.println("The fitness of the best feasible solution: " + Arrays.toString(bestFeasibleSolutionFitness));

/*        // Find the best feasible solution from all generations
        SolutionFitnessPair bestFeasibleSolutionFitnessPair = findBestFeasibleSolution(new ArrayList<>(bestSolutionsAcrossGenerations));
        int[] bestFeasibleSolution = bestFeasibleSolutionFitnessPair.getSolution();
        System.out.println("Best feasible solution across all generations: " + Arrays.toString(bestFeasibleSolution));
        System.out.println("The fitness of the best feasible solution: " + Arrays.toString(bestFeasibleSolutionFitnessPair.getFitness()));*/

        // Calculate and print the performance indicators
        SolutionIndicatorData indicatorData = new SolutionIndicatorData(bestFeasibleSolution);
        SolutionFitnessPair finalSolution = calculateFitness(bestFeasibleSolution, indicatorData, true);
        printPerformanceIndicators(bestFeasibleSolution, indicatorData, outputFile + "/trip_statistics.csv");

        // Print the NUMBER_OF_TRIPS_LONGER_THAN
        //System.out.println("Threshold for trips longer than " + THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING + ": " + NUMBER_OF_TRIPS_LONGER_TAHN);

        return new double[]{BUFFER_END_TIME, SEARCH_RADIUS_ORIGIN, SEARCH_RADIUS_DESTINATION, bestFeasibleSolutionFitness[0]};
    }

    // GA solver with NSGA-II modifications==============================================================================
    private List<SolutionFitnessPair> evolvePopulation(List<SolutionFitnessPair> population, int currentGeneration) {
        // Apply local search to improve the population after NSGA-II operations, and before offspring generation
        if (ENABLE_LOCAL_SEARCH) {
            population = localSearch(population, currentGeneration);
        }

        List<SolutionFitnessPair> newPop = new ArrayList<>();

        // Generate new offspring by crossover and mutation
        while (newPop.size() < POP_SIZE) {
            int[] parent1 = selectParent(population);
            int[] parent2 = selectParent(population);
            int[] child;

            if (rand.nextDouble() < CROSSOVER_RATE) {
                child = crossover(parent1, parent2);
            } else {
                child = rand.nextBoolean() ? parent1 : parent2; // Skip crossover, use parent directly
            }

            child = mutate(child); // Mutation is always applied
            SolutionFitnessPair solution = calculateFitness(child, null, false);
            newPop.add(solution);
        }

        // Combine the old population and the new population
        List<SolutionFitnessPair> combinedPop = new ArrayList<>(population);
        combinedPop.addAll(newPop);

        // Apply local search to improve the combined population
        //combinedPop = localSearch(combinedPop);

        // Perform non-dominated sorting on the improved population
        List<List<SolutionFitnessPair>> fronts = nonDominatedSort(combinedPop);

        // Select the next generation from the sorted fronts
        List<SolutionFitnessPair> nextGeneration = new ArrayList<>();
        for (List<SolutionFitnessPair> front : fronts) {
            calculateCrowdingDistance(front);
            if (nextGeneration.size() + front.size() <= POP_SIZE) {
                nextGeneration.addAll(front);
            } else {
                front.sort(Comparator.comparingInt(SolutionFitnessPair::getRank)
                        .thenComparingDouble(p -> p.getCrowdingDistance()).reversed());
                nextGeneration.addAll(front.subList(0, POP_SIZE - nextGeneration.size()));
                break;
            }
        }

        // Update the best solutions queue
        for (SolutionFitnessPair solution : nextGeneration) {
            if (bestSolutionsAcrossGenerations.size() < MAX_BEST_SOLUTIONS) {
                bestSolutionsAcrossGenerations.offer(solution);
            } else if (solution.getFitness()[0] > bestSolutionsAcrossGenerations.peek().getFitness()[0]) {
                bestSolutionsAcrossGenerations.poll(); // Remove the lowest fitness solution
                bestSolutionsAcrossGenerations.offer(solution);
            }
        }

        return nextGeneration;
    }

    // Initialize population with random assignments
    private List<SolutionFitnessPair> initializePopulation() {
        List<SolutionFitnessPair> population = new ArrayList<>();
        for (int i = 0; i < MultiObjectiveNSGAII.POP_SIZE; i++) {
            int[] individual = generateIndividual();
            SolutionFitnessPair solution = calculateFitness(individual, null, false);
            population.add(solution);
        }
        return population;
    }

    // Generate a random individual
    private int[] generateIndividual() {
        int[] individual = new int[subTrips.size()];
        //resetVehicleCapacities(tripVehicleMap); // Reset the vehicle capacity since capacity of vehicles will be updated during each individual generation
        //resetVehicleOccupancy(vehicleOccupancyMap);
        for (int i = 0; i < individual.length; i++) {
            assignAvailableVehicle(i, individual);
        }
        return individual;
    }

    // Selection - Tournament selection with rank and crowding distance
    private int[] selectParent(List<SolutionFitnessPair> population) {
        List<SolutionFitnessPair> tournament = new ArrayList<>();
        for (int i = 0; i < TOURNAMENT_SIZE; i++) {
            tournament.add(population.get(rand.nextInt(population.size())));
        }
        tournament.sort(Comparator.comparingInt(SolutionFitnessPair::getRank)
                .thenComparingDouble(p -> p.getCrowdingDistance()).reversed()); // Sort by rank and then by crowding distance
        return tournament.get(0).getSolution();
    }

    //TODO: consider to repair the solution if it is not feasible due to the violation of vehicle capacity constraint
    // Crossover - Single point crossover //TODO: Implement other types of crossover instead of single point
    private int[] crossover(int[] parent1, int[] parent2) {
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
    private int[] mutate(int[] individual) {
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
    private void assignAvailableVehicle(int i, int[] individual) {
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
            if (vehicleList == null || vehicleList.isEmpty()) {
                if (vehicleList == null) {
                    vehicleList = new ArrayList<>();
                }
                UAMVehicle newVehicle = feedDataForVehicleCreation(trip, false);
                vehicleList.add(newVehicle);
                tripVehicleMap.put(trip.getTripId(), vehicleList);
                //vehicleOccupancyMap.put(vehicle, VEHICLE_CAPACITY);

                // Update tripVehicleMap for all relevant trips
                updateTripVehicleMapForNewVehicle(newVehicle);
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
    private void updateTripVehicleMapForNewVehicle(UAMVehicle newVehicle) {
        UAMStation originStation = vehicleOriginStationMap.get(newVehicle.getId());
        UAMStation destinationStation = vehicleDestinationStationMap.get(newVehicle.getId());

        for (UAMTrip trip : subTrips) {
            if (trip.calculateAccessTeleportationDistance(originStation) <= SEARCH_RADIUS_ORIGIN &&
                    trip.calculateEgressTeleportationDistance(destinationStation) <= SEARCH_RADIUS_DESTINATION) {

                List<UAMVehicle> tripVehicles = tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>());
                tripVehicles.add(newVehicle);
                tripVehicleMap.put(trip.getTripId(), tripVehicles);
            }
        }
    }

    // Objective function ==============================================================================================
    // Calculate fitness for an individual
    private SolutionFitnessPair calculateFitness(int[] individual, SolutionIndicatorData indicatorData, boolean isFinalSolutions) {
        Map<Integer, List<UAMTrip>> vehicleAssignments = new HashMap<>();
        Map<Integer, Integer> vehicleLoadCount = new HashMap<>();
        Map<String, Double> travelTimeChangeMap = new HashMap<>();

        // Organize trips by assigned vehicle
        for (int i = 0; i < individual.length; i++) {
            int vehicleId = individual[i];
            /*            if (vehicleId == VALUE_FOR_NO_VEHICLE_AVAILABLE) {
                continue;
            }*/
            vehicleAssignments.computeIfAbsent(vehicleId, k -> new ArrayList<>()).add(subTrips.get(i));

            // Update vehicle load count
            vehicleLoadCount.put(vehicleId, vehicleLoadCount.getOrDefault(vehicleId, 0) + 1);
        }

        double[] fitness = getFitnessPerVehicle(isFinalSolutions, vehicleAssignments, travelTimeChangeMap, indicatorData);

        // Calculate pooling rate and vehicle capacity rates
        if (isFinalSolutions) {
            int pooledTrips = 0;
            int totalVehicles = vehicleAssignments.size();
            Map<Integer, Integer> capacityCount = new HashMap<>();
            for (List<UAMTrip> trips : vehicleAssignments.values()) {
                int tripCount = trips.size();
                if (tripCount > 1) {
                    pooledTrips += tripCount;
                }
                capacityCount.put(tripCount, capacityCount.getOrDefault(tripCount, 0) + 1);
            }
            indicatorData.setPoolingRate((double) pooledTrips / subTrips.size());

            for (int capacity = 0; capacity <= VEHICLE_CAPACITY; capacity++) {
                int count = capacityCount.getOrDefault(capacity, 0);
                double rate = (double) count / totalVehicles;
                indicatorData.getVehicleCapacityRates().put(capacity, rate);
            }

            // Calculate shared ride statistics
            List<Double> sharedTravelTimeChanges = new ArrayList<>();
            int sharedRidesExceedingThreshold = 0;
            for (List<UAMTrip> trips : vehicleAssignments.values()) {
                if (trips.size() > 1) {
                    for (UAMTrip trip : trips) {
                        double travelTimeChange = indicatorData.getTravelTimeChanges().get(trip.getTripId());
                        sharedTravelTimeChanges.add(travelTimeChange);
                        if (travelTimeChange > SHARED_RIDE_TRAVEL_TIME_CHANGE_THRESHOLD) {
                            sharedRidesExceedingThreshold++;
                        }
                    }
                }
            }

            int totalSharedRides = sharedTravelTimeChanges.size();
            indicatorData.setSharedRidesExceedingThresholdRate(totalSharedRides == 0 ? 0 : (double) sharedRidesExceedingThreshold / totalSharedRides);
            indicatorData.setTotalSharedRidesExceedingThresholdRate((double) sharedRidesExceedingThreshold / subTrips.size());

            // Store fitness in indicatorData
            indicatorData.setFitness(fitness);
        }

        // Store vehicleLoadCount and travelTimeChangeMap in the solution pair
        SolutionFitnessPair solutionPair = new SolutionFitnessPair(individual, fitness, vehicleLoadCount, travelTimeChangeMap);

        return solutionPair;
    }
    private double[] getFitnessPerVehicle(boolean isFinalSolutions, Map<Integer, List<UAMTrip>> vehicleAssignments, Map<String, Double> travelTimeChangeMap, SolutionIndicatorData indicatorData) {
        double totalFitness = 0.0;
        double totalDistanceChange = 0.0;
        double totalTimeChange = 0.0;
        double totalViolationPenalty = 0.0;

        // Calculate fitness per vehicle
        for (Map.Entry<Integer, List<UAMTrip>> entry : vehicleAssignments.entrySet()) {
            List<UAMTrip> trips = entry.getValue();
            UAMStation originStationOfVehicle = vehicleOriginStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));
            UAMStation destinationStationOfVehicle = vehicleDestinationStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));

            // safety check
            if (trips.isEmpty()) continue;
            if (trips.size() == 1){
                UAMTrip trip = trips.get(0);
                totalFitness = getFitnessForNonPooledOrBaseTrip(trip, originStationOfVehicle, destinationStationOfVehicle, totalFitness, isFinalSolutions, travelTimeChangeMap, indicatorData);
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
                    totalFitness = getFitnessForNonPooledOrBaseTrip(trip, originStationOfVehicle, destinationStationOfVehicle, totalFitness, isFinalSolutions, travelTimeChangeMap, indicatorData);
                    continue;
                }

                double tripTimeChange = 0.0;
                double tripFlightDistanceChange = 0.0;
                double tripTotalTravelTime = 0.0;

                // calculate change in flight distance
                double flightDistanceChange = getFlightDistanceChange(trip, originStationOfVehicle, destinationStationOfVehicle);
                totalFitness += ALPHA * flightDistanceChange;
                tripFlightDistanceChange += flightDistanceChange;
                // calculate saved flight distance
                double savedFlightDistance = trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation());
                totalFitness += ALPHA * (-1) * savedFlightDistance;
                tripFlightDistanceChange -= savedFlightDistance;
                if(isFinalSolutions){
                    //finalSolutionFlightDistanceChanges.put(trip.getTripId(), tripFlightDistanceChange);
                    indicatorData.setFlightDistanceChanges(trip.getTripId(), tripFlightDistanceChange);
                }
                // calculate change in flight time due to the change in flight distance
                double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
                totalFitness += BETA * flightTimeChange;
                tripTimeChange += flightTimeChange;
                // calculate additional travel time
                double originalArrivalTimeForThePooledTrip = trip.getDepartureTime() + trip.calculateAccessTeleportationTime(trip.getOriginStation());
                double travelTimeChangeDueToAccessMatching = boardingTimeForAllTrips - originalArrivalTimeForThePooledTrip;
                tripTimeChange += travelTimeChangeDueToAccessMatching;
                if(travelTimeChangeDueToAccessMatching > 0) {
                    totalFitness += BETA * travelTimeChangeDueToAccessMatching;
                } else {
                    totalFitness += BETA * (- travelTimeChangeDueToAccessMatching); //TODO: reconsider for "negative additional travel time" cases
                }
                double additionalTravelTimeDueToEgressMatching = getTravelTimeChangeDueToEgressMatching(trip, destinationStationOfVehicle);
                totalFitness += BETA * additionalTravelTimeDueToEgressMatching;
                tripTimeChange += additionalTravelTimeDueToEgressMatching;

                if(isFinalSolutions){
                    //finalSolutionTravelTimeChanges.put(trip.getTripId(), tripTimeChange);
                    indicatorData.setTravelTimeChanges(trip.getTripId(), tripTimeChange);
                }

                travelTimeChangeMap.put(trip.getTripId(), tripTimeChange);

                if(isFinalSolutions){
                    double departureRedirectionRate = ( trip.calculateAccessTeleportationDistance(originStationOfVehicle) - trip.calculateAccessTeleportationDistance(trip.getOriginStation()) ) / ( trip.calculateAccessTeleportationDistance(trip.getOriginStation()) );
                    //finalSolutionDepartureRedirectionRate.put(trip.getTripId(), departureRedirectionRate);
                    indicatorData.setDepartureRedirectionRate(trip.getTripId(), departureRedirectionRate);
                    double arrivalRedirectionRate = ( trip.calculateEgressTeleportationDistance(destinationStationOfVehicle) - trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) ) / ( trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) );
                    //finalSolutionArrivalRedirectionRate.put(trip.getTripId(), arrivalRedirectionRate);
                    indicatorData.setArrivalRedirectionRate(trip.getTripId(), arrivalRedirectionRate);

                    // total travel time for the trip
                    //TODO: Should the accessTime = boardingTimeForAllTrips - trip.getDepartureTime()?
                    tripTotalTravelTime = trip.calculateAccessTeleportationTime(originStationOfVehicle) + trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) / VEHICLE_CRUISE_SPEED + trip.calculateEgressTeleportationTime(destinationStationOfVehicle);
                    //finalSolutionTotalTravelTime.put(trip.getTripId(), tripTotalTravelTime);
                    indicatorData.setTotalTravelTime(trip.getTripId(), tripTotalTravelTime);

                    // assigned origin station
                    //finalSolutionAssignedAccessStation.put(trip.getTripId(), originStationOfVehicle.getId().toString());
                    indicatorData.setAssignedAccessStation(trip.getTripId(), originStationOfVehicle.getId().toString());
                    // assigned destination station
                    //finalSolutionAssignedEgressStation.put(trip.getTripId(), destinationStationOfVehicle.getId().toString());
                    indicatorData.setAssignedEgressStation(trip.getTripId(), destinationStationOfVehicle.getId().toString());
                }
                totalDistanceChange += tripFlightDistanceChange;
                totalTimeChange += tripTimeChange;
            }
            //add penalty for the case when vehicle capacity is violated
            if(trips.size() > VEHICLE_CAPACITY){
                totalViolationPenalty += PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION * (trips.size() - VEHICLE_CAPACITY);
            }
        }
        return new double[]{totalFitness, totalDistanceChange, totalTimeChange, totalViolationPenalty};
    }
    private double getFitnessForNonPooledOrBaseTrip(UAMTrip trip, UAMStation originStationOfVehicle, UAMStation destinationStationOfVehicle, double totalFitness, boolean isFinalSolutions, Map<String, Double> travelTimeChangeMap, SolutionIndicatorData indicatorData) {
        double tripTimeChange = 0.0;
        double tripFlightDistanceChange = 0.0;

        // calculate change in flight distance
        double flightDistanceChange = getFlightDistanceChange(trip, originStationOfVehicle, destinationStationOfVehicle);
        totalFitness += ALPHA * flightDistanceChange;
        tripFlightDistanceChange += flightDistanceChange;
        if(isFinalSolutions){
            //finalSolutionFlightDistanceChanges.put(trip.getTripId(), tripFlightDistanceChange);
            indicatorData.setFlightDistanceChanges(trip.getTripId(), tripFlightDistanceChange);
        }
        // calculate change in flight time due to the change in flight distance
        double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
        totalFitness += BETA * flightTimeChange;
        tripTimeChange += flightTimeChange;
        // calculate change in travel time due to access matching
        double travelTimeChangeDueToAccessMatching = trip.calculateAccessTeleportationTime(originStationOfVehicle) - trip.calculateAccessTeleportationTime(trip.getOriginStation());
        tripTimeChange += travelTimeChangeDueToAccessMatching;
        if(travelTimeChangeDueToAccessMatching > 0) {
            totalFitness += BETA * travelTimeChangeDueToAccessMatching;
        } else {
            totalFitness += BETA * (- travelTimeChangeDueToAccessMatching);
        }
        // calculate change in travel time due to egress matching
        double travelTimeChangeDueToEgressMatching = getTravelTimeChangeDueToEgressMatching(trip, destinationStationOfVehicle);
        totalFitness += BETA * travelTimeChangeDueToEgressMatching;
        tripTimeChange += travelTimeChangeDueToEgressMatching;

        if(isFinalSolutions){
            //finalSolutionTravelTimeChanges.put(trip.getTripId(), tripTimeChange);
            indicatorData.setTravelTimeChanges(trip.getTripId(), tripTimeChange);
        }

        travelTimeChangeMap.put(trip.getTripId(), tripTimeChange);

        if(isFinalSolutions){
            double departureRedirectionRate = ( trip.calculateAccessTeleportationDistance(originStationOfVehicle) - trip.calculateAccessTeleportationDistance(trip.getOriginStation()) ) / ( trip.calculateAccessTeleportationDistance(trip.getOriginStation()) );
            //finalSolutionDepartureRedirectionRate.put(trip.getTripId(), departureRedirectionRate);
            indicatorData.setDepartureRedirectionRate(trip.getTripId(), departureRedirectionRate);
            double arrivalRedirectionRate = ( trip.calculateEgressTeleportationDistance(destinationStationOfVehicle) - trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) ) / ( trip.calculateEgressTeleportationDistance(trip.getDestinationStation()) );
            //finalSolutionArrivalRedirectionRate.put(trip.getTripId(), arrivalRedirectionRate);
            indicatorData.setArrivalRedirectionRate(trip.getTripId(), arrivalRedirectionRate);

            // total travel time for the trip
            double totalTravelTime = trip.calculateAccessTeleportationTime(originStationOfVehicle) + trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) / VEHICLE_CRUISE_SPEED + trip.calculateEgressTeleportationTime(destinationStationOfVehicle);
            //finalSolutionTotalTravelTime.put(trip.getTripId(), totalTravelTime);
            indicatorData.setTotalTravelTime(trip.getTripId(), totalTravelTime);

            // assigned origin station
            //finalSolutionAssignedAccessStation.put(trip.getTripId(), originStationOfVehicle.getId().toString());
            indicatorData.setAssignedAccessStation(trip.getTripId(), originStationOfVehicle.getId().toString());
            // assigned destination station
            //finalSolutionAssignedEgressStation.put(trip.getTripId(), destinationStationOfVehicle.getId().toString());
            indicatorData.setAssignedEgressStation(trip.getTripId(), destinationStationOfVehicle.getId().toString());
        }
        return totalFitness;
    }
    private static double getFlightDistanceChange(UAMTrip trip, UAMStation originStationOfVehicle, UAMStation destinationStationOfVehicle) {
        return trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) - trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation());
    }
    private static double getTravelTimeChangeDueToEgressMatching(UAMTrip trip, UAMStation destinationStationOfVehicle) {
        return trip.calculateEgressTeleportationTime(destinationStationOfVehicle) - trip.calculateEgressTeleportationTime(trip.getDestinationStation());
    }

    // Helper methods for GA and NSGA-II ======================================================================
    private static List<List<SolutionFitnessPair>> nonDominatedSort(List<SolutionFitnessPair> population) {
        List<List<SolutionFitnessPair>> fronts = new ArrayList<>();
        Map<SolutionFitnessPair, List<SolutionFitnessPair>> dominationMap = new HashMap<>();
        Map<SolutionFitnessPair, Integer> dominatedCount = new HashMap<>();

        for (SolutionFitnessPair p : population) {
            dominationMap.put(p, new ArrayList<>());
            dominatedCount.put(p, 0);
            for (SolutionFitnessPair q : population) {
                if (dominates(p, q)) {
                    dominationMap.get(p).add(q);
                } else if (dominates(q, p)) {
                    dominatedCount.put(p, dominatedCount.get(p) + 1);
                }
            }
            if (dominatedCount.get(p) == 0) {
                p.rank = 0;
                if (fronts.isEmpty()) {
                    fronts.add(new ArrayList<>());
                }
                fronts.get(0).add(p);
            }
        }

        int i = 0;
        while (i < fronts.size() && !fronts.get(i).isEmpty()) {
            List<SolutionFitnessPair> nextFront = new ArrayList<>();
            for (SolutionFitnessPair p : fronts.get(i)) {
                for (SolutionFitnessPair q : dominationMap.get(p)) {
                    dominatedCount.put(q, dominatedCount.get(q) - 1);
                    if (dominatedCount.get(q) == 0) {
                        q.rank = i + 1;
                        nextFront.add(q);
                    }
                }
            }
            if (!nextFront.isEmpty()) {
                fronts.add(nextFront);
            }
            i++;
        }
        return fronts;
    }

    private Map<String, List<UAMVehicle>> findNearbyVehiclesToTrips(List<UAMTrip> subTrips) {
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

    private static boolean dominates(SolutionFitnessPair p, SolutionFitnessPair q) {
        boolean betterInAnyObjective = false;
        for (int i = 0; i < p.getFitness().length; i++) {
            if (p.getFitness()[i] < q.getFitness()[i]) {
                betterInAnyObjective = true;
            } else if (p.getFitness()[i] > q.getFitness()[i]) {
                return false;
            }
        }
        return betterInAnyObjective;
    }

    private static void calculateCrowdingDistance(List<SolutionFitnessPair> front) {
        int n = front.size();
        if (n == 0) return;

        for (SolutionFitnessPair p : front) {
            p.crowdingDistance = 0;
        }

        int m = front.get(0).getFitness().length;
        for (int i = 0; i < m; i++) {
            final int objIndex = i;
            front.sort(Comparator.comparingDouble(p -> p.getFitness()[objIndex]));
            front.get(0).crowdingDistance = Double.POSITIVE_INFINITY;
            front.get(n - 1).crowdingDistance = Double.POSITIVE_INFINITY;
            double minValue = front.get(0).getFitness()[objIndex];
            double maxValue = front.get(n - 1).getFitness()[objIndex];
            for (int j = 1; j < n - 1; j++) {
                front.get(j).crowdingDistance += (front.get(j + 1).getFitness()[objIndex] - front.get(j - 1).getFitness()[objIndex]) / (maxValue - minValue);
            }
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

    // Local search methods ============================================================================================
    // Ruin and recreate solution

    // Adjusted Ruin and Recreate Methods
    // Method to determine the number of trips to ruin based on the current generation
    private int determineRuinDegree(int currentGeneration, int maxGenerations) {
        // Start with higher ruin degree and gradually decrease
        double ruinFactor = (1.0 - ((double) currentGeneration / maxGenerations));
        return (int) Math.ceil(ruinFactor * subTrips.size() / 2);
    }
    /*private static int[] ruinSolution(int[] solution, int currentGeneration, int maxGenerations) {
        int[] ruinedSolution = Arrays.copyOf(solution, solution.length);

        int numTripsToRuin = determineRuinDegree(currentGeneration, maxGenerations);

        Set<Integer> selectedIndices = new HashSet<>();
        while (selectedIndices.size() < numTripsToRuin) {
            int tripIndex = rand.nextInt(ruinedSolution.length);
            if (!selectedIndices.contains(tripIndex)) {
                selectedIndices.add(tripIndex);
                ruinedSolution[tripIndex] = VALUE_FOR_NO_VEHICLE_AVAILABLE; // Mark the trip as unassigned
            }
        }

        return ruinedSolution;
    }
    private static List<SolutionFitnessPair> localSearch(List<SolutionFitnessPair> population, int currentGeneration) {
        int maxGenerations = 100;
        List<SolutionFitnessPair> improvedPopulation = new ArrayList<>();

        for (SolutionFitnessPair solutionPair : population) {
            int[] currentSolution = solutionPair.getSolution();
            double[] currentFitness = solutionPair.getFitness();

            int[] bestSolution = Arrays.copyOf(currentSolution, currentSolution.length);
            double[] bestFitness = Arrays.copyOf(currentFitness, currentFitness.length);

            for (int i = 0; i < maxGenerations; i++) { // Number of iterations for local search
                int[] ruinedSolution = ruinSolution(bestSolution, currentGeneration, maxGenerations);
                int[] recreatedSolution = recreateSolution(ruinedSolution);
                double[] recreatedFitness = calculateFitness(recreatedSolution, false);

                if (dominates(new SolutionFitnessPair(recreatedSolution, recreatedFitness), new SolutionFitnessPair(bestSolution, bestFitness))) {
                    bestSolution = recreatedSolution;
                    bestFitness = recreatedFitness;
                }
            }

            improvedPopulation.add(new SolutionFitnessPair(bestSolution, bestFitness));
        }

        return improvedPopulation;
    }*/
    // Targeted Ruin - Focus on trips that are likely to improve the solution
    private int[] targetedRuin(SolutionFitnessPair solutionPair, int currentGeneration, int maxGenerations) {
        int[] ruinedSolution = solutionPair.getSolution();

        // Reset the vehicle load count and travel time change map
        Map<Integer, Integer> vehicleLoadCount = solutionPair.getVehicleLoadCount();
        Map<String, Double> travelTimeChangeMap = solutionPair.getTravelTimeChangeMap();

        List<Integer> targetTrips = new ArrayList<>();

        for (int i = 0; i < ruinedSolution.length; i++) {
            int vehicleId = ruinedSolution[i];

            if ((vehicleLoadCount.containsKey(vehicleId) && vehicleLoadCount.get(vehicleId) > VEHICLE_CAPACITY) || travelTimeChangeMap.get(subTrips.get(i).getTripId()) > SHARED_RIDE_TRAVEL_TIME_CHANGE_THRESHOLD) {
                targetTrips.add(i);
            }
        }

        // Randomly select trips to ruin from the targeted trips
        int numTripsToRuin = Math.min(targetTrips.size(), determineRuinDegree(currentGeneration, maxGenerations));  // Adjust as needed
        Collections.shuffle(targetTrips);

        for (int i = 0; i < numTripsToRuin; i++) {
            int tripIndex = targetTrips.get(i);
            ruinedSolution[tripIndex] = VALUE_FOR_NO_VEHICLE_AVAILABLE; // Mark the trip as unassigned
        }

        return ruinedSolution;
    }
    private int[] recreateSolution(int[] ruinedSolution) {
        int[] recreatedSolution = Arrays.copyOf(ruinedSolution, ruinedSolution.length);

        for (int i = 0; i < recreatedSolution.length; i++) {
            if (recreatedSolution[i] == VALUE_FOR_NO_VEHICLE_AVAILABLE) {
                assignAvailableVehicle(i, recreatedSolution);
            }
        }

        return recreatedSolution;
    }
    private List<SolutionFitnessPair> localSearch(List<SolutionFitnessPair> population, int currentGeneration) {
        int maxGenerations = 100;

        List<SolutionFitnessPair> improvedPopulation = new ArrayList<>();

        for (SolutionFitnessPair solutionPair : population) {

            for (int i = 0; i < maxGenerations; i++) { // Number of iterations for local search
                int[] ruinedSolution = targetedRuin(solutionPair, currentGeneration, maxGenerations);
                int[] recreatedSolution = recreateSolution(ruinedSolution);
                SolutionFitnessPair newSolution = calculateFitness(recreatedSolution, null, false);

                if (!dominates(newSolution, solutionPair)) {
                    solutionPair = newSolution;
                }
            }

            improvedPopulation.add(solutionPair);
        }

        return improvedPopulation;
    }

    // SolutionFitnessPair related methods =============================================================================
    // SolutionFitnessPair class to hold individual solutions and their fitness, along with vehicleLoadCount and travelTimeChangeMap
    private static class SolutionFitnessPair {
        private final int[] solution;
        private final double[] fitness;
        private final Map<Integer, Integer> vehicleLoadCount;
        private final Map<String, Double> travelTimeChangeMap;
        private int rank;
        private double crowdingDistance;

        public SolutionFitnessPair(int[] solution, double[] fitness) {
            this.solution = solution;
            this.fitness = fitness;
            this.vehicleLoadCount = null;
            this.travelTimeChangeMap = null;
            this.rank = Integer.MAX_VALUE;
            this.crowdingDistance = 0;
        }

        public SolutionFitnessPair(int[] solution, double[] fitness, Map<Integer, Integer> vehicleLoadCount, Map<String, Double> travelTimeChangeMap) {
            this.solution = solution;
            this.fitness = fitness;
            this.vehicleLoadCount = vehicleLoadCount;
            this.travelTimeChangeMap = travelTimeChangeMap;
            this.rank = Integer.MAX_VALUE;
            this.crowdingDistance = 0;
        }

        public int[] getSolution() {
            return solution;
        }

        public double[] getFitness() {
            return fitness;
        }

        public int getRank() {
            return rank;
        }

        public double getCrowdingDistance() {
            return crowdingDistance;
        }

        public Map<Integer, Integer> getVehicleLoadCount() {
            return vehicleLoadCount;
        }

        public Map<String, Double> getTravelTimeChangeMap() {
            return travelTimeChangeMap;
        }
    }

    // Method to find the first feasible solution from the priority queue without altering the original heap
    private SolutionFitnessPair findBestFeasibleSolution(List<SolutionFitnessPair> population) {
        // Create a new priority queue that is a copy of the original but sorted in descending order by fitness
        PriorityQueue<SolutionFitnessPair> solutionsHeapCopy = new PriorityQueue<>(
                Comparator.comparingDouble((SolutionFitnessPair p) -> p.getFitness()[0]).reversed()
        );
        solutionsHeapCopy.addAll(population);
        SolutionFitnessPair bestSolutionButMaybeInfeasible = solutionsHeapCopy.peek();

        // Iterate through the copied solutions heap to find a feasible solution
        while (!solutionsHeapCopy.isEmpty()) {
            SolutionFitnessPair solutionPair = solutionsHeapCopy.poll(); // Remove and retrieve the solution with the highest fitness
            int[] candidateSolution = solutionPair.getSolution();
            if (isFeasible(candidateSolution, false)) {
                return solutionPair;
            }
        }
        int[] quickFixedSolution = guaranteeFeasibleSolution(bestSolutionButMaybeInfeasible.getSolution());
        return new SolutionFitnessPair(quickFixedSolution, calculateFitness(quickFixedSolution, null, false).getFitness());
        //throw new IllegalStateException("No feasible solution found in the entire population");
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

    public int[] guaranteeFeasibleSolution(int[] solution) {
        boolean isSolutionFeasible = false;
        int iterationCount = 0;
        final int MAX_ITERATIONS = 1000000; // A very high number, but not infinite

        while (!isSolutionFeasible && iterationCount < MAX_ITERATIONS) {
            Map<Integer, List<Integer>> vehicleAssignments = new HashMap<>();

            // Group trips by assigned vehicle
            for (int i = 0; i < solution.length; i++) {
                int vehicleId = solution[i];
                vehicleAssignments.computeIfAbsent(vehicleId, k -> new ArrayList<>()).add(i);
            }

            // Identify and fix overloaded vehicles
            for (Map.Entry<Integer, List<Integer>> entry : vehicleAssignments.entrySet()) {
                int vehicleId = entry.getKey();
                List<Integer> assignedTrips = entry.getValue();

                if (assignedTrips.size() > VEHICLE_CAPACITY) {
                    // Sort trips by arrival time at the station
                    assignedTrips.sort((a, b) -> {
                        UAMTrip tripA = subTrips.get(a);
                        UAMTrip tripB = subTrips.get(b);
                        UAMStation station = vehicleOriginStationMap.get(Id.create(String.valueOf(vehicleId), DvrpVehicle.class));
                        double arrivalTimeA = tripA.getDepartureTime() + tripA.calculateAccessTeleportationTime(station);
                        double arrivalTimeB = tripB.getDepartureTime() + tripB.calculateAccessTeleportationTime(station);
                        return Double.compare(arrivalTimeA, arrivalTimeB);
                    });

                    // Reassign extra trips
                    for (int i = VEHICLE_CAPACITY; i < assignedTrips.size(); i++) {
                        int tripIndex = assignedTrips.get(i);
                        UAMTrip trip = subTrips.get(tripIndex);

                        // Check if all available vehicles are at capacity
                        List<UAMVehicle> availableVehicles = new ArrayList<>(tripVehicleMap.get(trip.getTripId()));
                        boolean allVehiclesAtCapacity = availableVehicles.stream()
                                .allMatch(v -> vehicleAssignments.getOrDefault(Integer.parseInt(v.getId().toString()), Collections.emptyList()).size() >= VEHICLE_CAPACITY);

                        if (allVehiclesAtCapacity) {
                            // Create a new vehicle for this trip
                            UAMVehicle newVehicle = feedDataForVehicleCreation(trip, false);
                            availableVehicles.add(newVehicle);
                            tripVehicleMap.put(trip.getTripId(), availableVehicles);

                            // Update tripVehicleMap for the new vehicle
                            updateTripVehicleMapForNewVehicle(newVehicle);

                            // Assign the trip to the new vehicle
                            solution[tripIndex] = Integer.parseInt(newVehicle.getId().toString());
                        } else {
                            // Assign to an available vehicle that's not at capacity
                            assignAvailableVehicle(tripIndex, solution);
                        }
                    }
                }
            }

            // Check if the solution is now feasible
            isSolutionFeasible = isFeasible(solution, false);

            // If the solution is still infeasible
            // create new vehicles for all overloaded trips
            if (!isSolutionFeasible) {
                forceCreateNewVehicles(solution);
            }

            iterationCount++;
        }

        if (!isSolutionFeasible) {
            throw new IllegalArgumentException("Error: Could not find a feasible solution after " + MAX_ITERATIONS + " iterations.");
        }
        return solution;
    }

    private void forceCreateNewVehicles(int[] solution) {
        Map<Integer, List<Integer>> vehicleAssignments = new HashMap<>();

        // Group trips by assigned vehicle
        for (int i = 0; i < solution.length; i++) {
            int vehicleId = solution[i];
            vehicleAssignments.computeIfAbsent(vehicleId, k -> new ArrayList<>()).add(i);
        }

        for (Map.Entry<Integer, List<Integer>> entry : vehicleAssignments.entrySet()) {
            List<Integer> assignedTrips = entry.getValue();
            if (assignedTrips.size() > VEHICLE_CAPACITY) {
                for (int i = VEHICLE_CAPACITY; i < assignedTrips.size(); i++) {
                    int tripIndex = assignedTrips.get(i);
                    UAMTrip trip = subTrips.get(tripIndex);
                    UAMVehicle newVehicle = feedDataForVehicleCreation(trip, false);
                    List<UAMVehicle> availableVehicles = new ArrayList<>(tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>()));
                    availableVehicles.add(newVehicle);
                    tripVehicleMap.put(trip.getTripId(), availableVehicles);

                    // Update tripVehicleMap for the new vehicle
                    updateTripVehicleMapForNewVehicle(newVehicle);

                    solution[tripIndex] = Integer.parseInt(newVehicle.getId().toString());
                }
            }
        }
    }

    // Performance indicators ==========================================================================================
    // Method to calculate and print the performance indicators
    private void printPerformanceIndicators(int[] solution, SolutionIndicatorData indicatorData, String tripStatisticsCSVFile) {
        // Print the pooling rate
        System.out.println("Pooling rate: " + indicatorData.getPoolingRate());

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
        int sharedRidesExceedingThreshold = 0;  // Counter for shared rides with travel time changes exceeding threshold

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
                    if (indicatorData.getTravelTimeChanges().containsKey(tripId)) {
                        double travelTimeChange = indicatorData.getTravelTimeChanges().get(tripId);
                        sharedTravelTimeChanges.add(travelTimeChange);
                        if (travelTimeChange > SHARED_RIDE_TRAVEL_TIME_CHANGE_THRESHOLD) {
                            sharedRidesExceedingThreshold++;  // Increment counter
                        }
                    }
                }
            }
        }

        // Calculate and print the share of shared rides with travel time changes exceeding threshold
        int totalSharedRides = sharedTravelTimeChanges.size();
        double shareExceedingThreshold = totalSharedRides == 0 ? 0 : (double) sharedRidesExceedingThreshold / totalSharedRides;
        System.out.println("Share of shared rides with travel time changes exceeding" + SHARED_RIDE_TRAVEL_TIME_CHANGE_THRESHOLD + ": " + shareExceedingThreshold);
        double totalShareExceedingThreshold = subTrips.isEmpty() ? 0 : (double) sharedRidesExceedingThreshold / subTrips.size();
        System.out.println("Total share of shared rides with travel time changes exceeding" + SHARED_RIDE_TRAVEL_TIME_CHANGE_THRESHOLD + ": " + totalShareExceedingThreshold);

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

        Collection<Double> flightDistanceChanges = indicatorData.getFlightDistanceChanges().values();
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

        Collection<Double> departureRedirectionRates = indicatorData.getDepartureRedirectionRates().values();
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

        Collection<Double> arrivalRedirectionRates = indicatorData.getArrivalRedirectionRates().values();
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
        printStatisticsToCsv(solution, indicatorData, tripStatisticsCSVFile);
    }
    // Method to print statistics to a CSV file
    private void printStatisticsToCsv(int[] solution, SolutionIndicatorData indicatorData, String fileName) {
        // Create a map to count the number of trips assigned to each vehicle
        Map<Integer, Integer> vehicleTripCount = new HashMap<>();
        for (int vehicleId : solution) {
            vehicleTripCount.put(vehicleId, vehicleTripCount.getOrDefault(vehicleId, 0) + 1);
        }

        try (FileWriter writer = new FileWriter(fileName)) {
            writer.append("TripId,AssignedVehicleId,AccessStationId,EgressStationId,TotalTravelTime,TravelTimeChange,FlightDistanceChange,DepartureRedirectionRate,ArrivalRedirectionRate,VehicleTripCount\n");
            for (int i = 0; i < solution.length; i++) {
                UAMTrip trip = subTrips.get(i);
                String tripId = trip.getTripId();
                int assignedVehicleId = solution[i];
                double travelTimeChange = indicatorData.getTravelTimeChanges().get(tripId);
                double flightDistanceChange = indicatorData.getFlightDistanceChanges().get(tripId);
                double departureRedirectionRate = indicatorData.getDepartureRedirectionRates().get(tripId);
                double arrivalRedirectionRate = indicatorData.getArrivalRedirectionRates().get(tripId);
                String assignedAccessStation = indicatorData.getAssignedAccessStations().get(tripId);
                String assignedEgressStation = indicatorData.getAssignedEgressStations().get(tripId);
                double totalTravelTime = indicatorData.getTotalTravelTimes().get(tripId);

                // Get the number of trips assigned to the same vehicle
                int tripCountForVehicle = vehicleTripCount.getOrDefault(assignedVehicleId, 0);

                writer.append(String.format("%s,%d,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
                        tripId, assignedVehicleId, assignedAccessStation, assignedEgressStation, totalTravelTime, travelTimeChange, flightDistanceChange, departureRedirectionRate, arrivalRedirectionRate, tripCountForVehicle));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public class SolutionIndicatorData {
        private final int[] solution;
        private final Map<String, Double> travelTimeChanges = new HashMap<>();
        private final Map<String, Double> flightDistanceChanges = new HashMap<>();
        private final Map<String, Double> departureRedirectionRates = new HashMap<>();
        private final Map<String, Double> arrivalRedirectionRates = new HashMap<>();
        private final Map<String, Double> totalTravelTimes = new HashMap<>();
        private final Map<String, String> assignedAccessStations = new HashMap<>();
        private final Map<String, String> assignedEgressStations = new HashMap<>();

        private double[] fitness;
        private double poolingRate;
        private Map<Integer, Double> vehicleCapacityRates = new HashMap<>();
        private double sharedRidesExceedingThresholdRate;
        private double totalSharedRidesExceedingThresholdRate;
        private double averageTravelTimeChange;
        private double percentile5thTravelTimeChange;
        private double percentile95thTravelTimeChange;
        private double averageFlightDistanceChange;
        private double percentile5thFlightDistanceChange;
        private double percentile95thFlightDistanceChange;
        private double averageDepartureRedirectionRate;
        private double percentile5thDepartureRedirectionRate;
        private double percentile95thDepartureRedirectionRate;
        private double averageArrivalRedirectionRate;
        private double percentile5thArrivalRedirectionRate;
        private double percentile95thArrivalRedirectionRate;

        private double averageTotalTravelTime;
        private double percentile5thTotalTravelTime;
        private double percentile95thTotalTravelTime;

        public SolutionIndicatorData(int[] solution) {
            this.solution = solution;
        }

        // Getters and setters for all fields
        public int[] getSolution() { return solution; }
        private Map<String, Double> getTravelTimeChanges() { return travelTimeChanges; }
        private Map<String, Double> getFlightDistanceChanges() { return flightDistanceChanges; }
        private Map<String, Double> getDepartureRedirectionRates() { return departureRedirectionRates; }
        private Map<String, Double> getArrivalRedirectionRates() { return arrivalRedirectionRates; }
        private Map<String, Double> getTotalTravelTimes() { return totalTravelTimes; }
        private  Map<String, String> getAssignedAccessStations() { return assignedAccessStations; }
        private  Map<String, String> getAssignedEgressStations() { return assignedEgressStations; }
        public double[] getFitness() { return fitness; }
        public void setFitness(double[] fitness) { this.fitness = fitness; }
        public double getPoolingRate() { return poolingRate; }
        public void setPoolingRate(double poolingRate) { this.poolingRate = poolingRate; }
        public Map<Integer, Double> getVehicleCapacityRates() { return vehicleCapacityRates; }
        //public void setVehicleCapacityRates(Map<Integer, Double> vehicleCapacityRates) { this.vehicleCapacityRates = vehicleCapacityRates; }
        public double getSharedRidesExceedingThresholdRate() { return sharedRidesExceedingThresholdRate; }
        public void setSharedRidesExceedingThresholdRate(double sharedRidesExceedingThresholdRate) { this.sharedRidesExceedingThresholdRate = sharedRidesExceedingThresholdRate; }
        public double getTotalSharedRidesExceedingThresholdRate() { return totalSharedRidesExceedingThresholdRate; }
        public void setTotalSharedRidesExceedingThresholdRate(double totalSharedRidesExceedingThresholdRate) { this.totalSharedRidesExceedingThresholdRate = totalSharedRidesExceedingThresholdRate; }
        public double getAverageTravelTimeChange() { return averageTravelTimeChange; }
        public void setAverageTravelTimeChange(double averageTravelTimeChange) { this.averageTravelTimeChange = averageTravelTimeChange; }
        public double getPercentile5thTravelTimeChange() { return percentile5thTravelTimeChange; }
        public void setPercentile5thTravelTimeChange(double percentile5thTravelTimeChange) { this.percentile5thTravelTimeChange = percentile5thTravelTimeChange; }
        public double getPercentile95thTravelTimeChange() { return percentile95thTravelTimeChange; }
        public void setPercentile95thTravelTimeChange(double percentile95thTravelTimeChange) { this.percentile95thTravelTimeChange = percentile95thTravelTimeChange; }
        public double getAverageFlightDistanceChange() { return averageFlightDistanceChange; }
        public void setAverageFlightDistanceChange(double averageFlightDistanceChange) { this.averageFlightDistanceChange = averageFlightDistanceChange; }
        public double getPercentile5thFlightDistanceChange() { return percentile5thFlightDistanceChange; }
        public void setPercentile5thFlightDistanceChange(double percentile5thFlightDistanceChange) { this.percentile5thFlightDistanceChange = percentile5thFlightDistanceChange; }
        public double getPercentile95thFlightDistanceChange() { return percentile95thFlightDistanceChange; }
        public void setPercentile95thFlightDistanceChange(double percentile95thFlightDistanceChange) { this.percentile95thFlightDistanceChange = percentile95thFlightDistanceChange; }
        public double getAverageDepartureRedirectionRate() { return averageDepartureRedirectionRate; }
        public void setAverageDepartureRedirectionRate(double averageDepartureRedirectionRate) { this.averageDepartureRedirectionRate = averageDepartureRedirectionRate; }
        public double getPercentile5thDepartureRedirectionRate() { return percentile5thDepartureRedirectionRate; }
        public void setPercentile5thDepartureRedirectionRate(double percentile5thDepartureRedirectionRate) { this.percentile5thDepartureRedirectionRate = percentile5thDepartureRedirectionRate; }
        public double getPercentile95thDepartureRedirectionRate() { return percentile95thDepartureRedirectionRate; }
        public void setPercentile95thDepartureRedirectionRate(double percentile95thDepartureRedirectionRate) { this.percentile95thDepartureRedirectionRate = percentile95thDepartureRedirectionRate; }
        public double getAverageArrivalRedirectionRate() { return averageArrivalRedirectionRate; }
        public void setAverageArrivalRedirectionRate(double averageArrivalRedirectionRate) { this.averageArrivalRedirectionRate = averageArrivalRedirectionRate; }
        public double getPercentile5thArrivalRedirectionRate() { return percentile5thArrivalRedirectionRate; }
        public void setPercentile5thArrivalRedirectionRate(double percentile5thArrivalRedirectionRate) { this.percentile5thArrivalRedirectionRate = percentile5thArrivalRedirectionRate; }
        public double getPercentile95thArrivalRedirectionRate() { return percentile95thArrivalRedirectionRate; }
        public void setPercentile95thArrivalRedirectionRate(double percentile95thArrivalRedirectionRate) { this.percentile95thArrivalRedirectionRate = percentile95thArrivalRedirectionRate; }

        public double getAverageTotalTravelTime() { return averageTotalTravelTime; }
        public void setAverageTotalTravelTime(double averageTotalTravelTime) { this.averageTotalTravelTime = averageTotalTravelTime; }
        public double getPercentile5thTotalTravelTime() { return percentile5thTotalTravelTime; }
        public void setPercentile5thTotalTravelTime(double percentile5thTotalTravelTime) { this.percentile5thTotalTravelTime = percentile5thTotalTravelTime; }
        public double getPercentile95thTotalTravelTime() { return percentile95thTotalTravelTime; }
        public void setPercentile95thTotalTravelTime(double percentile95thTotalTravelTime) { this.percentile95thTotalTravelTime = percentile95thTotalTravelTime; }

        // My adding: setArrivalRedirectionRate, setDepartureRedirectionRate, setTotalTravelTime, setAssignedAccessStation, setAssignedEgressStation
        public void setTravelTimeChanges(String tripId, Double travelTimeChanges) {
            this.travelTimeChanges.put(tripId, travelTimeChanges);
        }
        public void setFlightDistanceChanges(String tripId, Double flightDistanceChanges) {
            this.flightDistanceChanges.put(tripId, flightDistanceChanges);
        }
        public void setDepartureRedirectionRate(String tripId, Double departureRedirectionRates) {
            this.departureRedirectionRates.put(tripId, departureRedirectionRates);
        }
        public void setArrivalRedirectionRate(String tripId, Double arrivalRedirectionRates) {
            this.arrivalRedirectionRates.put(tripId, arrivalRedirectionRates);
        }
        public void setTotalTravelTime(String tripId, Double totalTravelTimes) {
            this.totalTravelTimes.put(tripId, totalTravelTimes);
        }
        public void setAssignedAccessStation(String tripId, String assignedAccessStations) {
            this.assignedAccessStations.put(tripId, assignedAccessStations);
        }
        public void setAssignedEgressStation(String tripId, String assignedEgressStations) {
            this.assignedEgressStations.put(tripId, assignedEgressStations);
        }
    }
    private void calculatePopulationIndicators(List<SolutionFitnessPair> population) {
        List<SolutionIndicatorData> indicatorDataList = new ArrayList<>();

        for (SolutionFitnessPair solutionPair : population) {
            SolutionIndicatorData indicatorData = new SolutionIndicatorData(solutionPair.getSolution());
            calculateFitness(solutionPair.getSolution(), indicatorData, true);
            calculateAdditionalIndicators(indicatorData);
            indicatorDataList.add(indicatorData);
        }

        // Write indicators to CSV
        writeIndicatorsToCsv(indicatorDataList, outputFile + "/last_iteration_solutions_indicators.csv");
    }
    private void calculateAdditionalIndicators(SolutionIndicatorData indicatorData) {
        List<Double> travelTimeChanges = new ArrayList<>(indicatorData.getTravelTimeChanges().values());
        List<Double> flightDistanceChanges = new ArrayList<>(indicatorData.getFlightDistanceChanges().values());
        List<Double> departureRedirectionRates = new ArrayList<>(indicatorData.getDepartureRedirectionRates().values());
        List<Double> arrivalRedirectionRates = new ArrayList<>(indicatorData.getArrivalRedirectionRates().values());
        List<Double> totalTravelTimes = new ArrayList<>(indicatorData.getTotalTravelTimes().values());

        // Sort lists for percentile calculations
        Collections.sort(travelTimeChanges);
        Collections.sort(flightDistanceChanges);
        Collections.sort(departureRedirectionRates);
        Collections.sort(arrivalRedirectionRates);
        Collections.sort(totalTravelTimes);

        // Calculate averages
        indicatorData.setAverageTravelTimeChange(calculateAverage(travelTimeChanges));
        indicatorData.setAverageFlightDistanceChange(calculateAverage(flightDistanceChanges));
        indicatorData.setAverageDepartureRedirectionRate(calculateAverage(departureRedirectionRates));
        indicatorData.setAverageArrivalRedirectionRate(calculateAverage(arrivalRedirectionRates));
        indicatorData.setAverageTotalTravelTime(calculateAverage(totalTravelTimes));

        // Calculate percentiles
        indicatorData.setPercentile5thTravelTimeChange(calculatePercentile(travelTimeChanges, 5));
        indicatorData.setPercentile95thTravelTimeChange(calculatePercentile(travelTimeChanges, 95));
        indicatorData.setPercentile5thFlightDistanceChange(calculatePercentile(flightDistanceChanges, 5));
        indicatorData.setPercentile95thFlightDistanceChange(calculatePercentile(flightDistanceChanges, 95));
        indicatorData.setPercentile5thDepartureRedirectionRate(calculatePercentile(departureRedirectionRates, 5));
        indicatorData.setPercentile95thDepartureRedirectionRate(calculatePercentile(departureRedirectionRates, 95));
        indicatorData.setPercentile5thArrivalRedirectionRate(calculatePercentile(arrivalRedirectionRates, 5));
        indicatorData.setPercentile95thArrivalRedirectionRate(calculatePercentile(arrivalRedirectionRates, 95));
        indicatorData.setPercentile5thTotalTravelTime(calculatePercentile(totalTravelTimes, 5));
        indicatorData.setPercentile95thTotalTravelTime(calculatePercentile(totalTravelTimes, 95));
    }
    private double calculateAverage(List<Double> values) {
        return values.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
    }
    private double calculatePercentile(List<Double> sortedValues, int percentile) {
        if (sortedValues.isEmpty()) {
            return Double.NaN;
        }
        int index = (int) Math.ceil(percentile / 100.0 * sortedValues.size()) - 1;
        return sortedValues.get(Math.max(0, Math.min(sortedValues.size() - 1, index)));
    }
    private void writeIndicatorsToCsv(List<SolutionIndicatorData> indicatorDataList, String fileName) {
        try (FileWriter writer = new FileWriter(fileName)) {
            // Write header
            writer.append("TotalFitness,TotalFlightDistanceChange,TotalTravelTimeChange,TotalCapacityViolationPenalty,PoolingRate,Capacity0Rate,Capacity1Rate,Capacity2Rate,Capacity3Rate,Capacity4Rate,SharedRidesExceedingThresholdRate,TotalSharedRidesExceedingThresholdRate,AvgTravelTimeChange,5thPercentileTravelTimeChange,95thPercentileTravelTimeChange,AvgFlightDistanceChange,5thPercentileFlightDistanceChange,95thPercentileFlightDistanceChange,AvgDepartureRedirectionRate,5thPercentileDepartureRedirectionRate,95thPercentileDepartureRedirectionRate,AvgArrivalRedirectionRate,5thPercentileArrivalRedirectionRate,95thPercentileArrivalRedirectionRate,AvgTotalTravelTime,5thPercentileTotalTravelTime,95thPercentileTotalTravelTime\n");

            // Write data for each solution
            for (SolutionIndicatorData data : indicatorDataList) {
                writer.append(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        data.getFitness()[0], data.getFitness()[1], data.getFitness()[2], data.getFitness()[3],
                        data.getPoolingRate(),
                        data.getVehicleCapacityRates().getOrDefault(0, 0.0),
                        data.getVehicleCapacityRates().getOrDefault(1, 0.0),
                        data.getVehicleCapacityRates().getOrDefault(2, 0.0),
                        data.getVehicleCapacityRates().getOrDefault(3, 0.0),
                        data.getVehicleCapacityRates().getOrDefault(4, 0.0),
                        data.getSharedRidesExceedingThresholdRate(),
                        data.getTotalSharedRidesExceedingThresholdRate(),
                        data.getAverageTravelTimeChange(),
                        data.getPercentile5thTravelTimeChange(),
                        data.getPercentile95thTravelTimeChange(),
                        data.getAverageFlightDistanceChange(),
                        data.getPercentile5thFlightDistanceChange(),
                        data.getPercentile95thFlightDistanceChange(),
                        data.getAverageDepartureRedirectionRate(),
                        data.getPercentile5thDepartureRedirectionRate(),
                        data.getPercentile95thDepartureRedirectionRate(),
                        data.getAverageArrivalRedirectionRate(),
                        data.getPercentile5thArrivalRedirectionRate(),
                        data.getPercentile95thArrivalRedirectionRate(),
                        data.getAverageTotalTravelTime(),
                        data.getPercentile5thTotalTravelTime(),
                        data.getPercentile95thTotalTravelTime()
                ));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Initial data extraction methods =================================================================================
    private static ArrayList<UAMTrip> extractSubTrips(List<UAMTrip> uamTrips) {
        // extract sub trips from uamTrips based on the departure time of trips falling between buffer start and end time
        return uamTrips.stream()
                .filter(trip -> trip.getDepartureTime() >= BUFFER_START_TIME && trip.getDepartureTime() < BUFFER_END_TIME)
                .collect(Collectors.toCollection(ArrayList::new));
    }

    // Method to create UAM vehicles and assign them to stations in the initialization phase (could also be used in later stage)
    private void saveStationVehicleNumber(List<UAMTrip> subTrips) {

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

    private UAMVehicle feedDataForVehicleCreation(UAMTrip subTrip, boolean isAddingVehicleBeforeInitialization) {
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
                    .map(MultiObjectiveNSGAII::parseTrip)
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

    private void repairInfeasibleSolutions(int numProcessors, ExecutorService executorService, ArrayBlockingQueue<SolutionFitnessPair> queue) throws InterruptedException {
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
                                repairedSolutionsHeap.add(calculateFitness(solution, null, false));
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

    private boolean fixInfeasibleSolutionWithORTools(int[] solution) {
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

            // Objective: Minimize the number of changes in vehicle assignments and prefer pooling
            int[] initialAssignments = Arrays.copyOf(solution, solution.length);
            IntVar[] assignmentChanges = new IntVar[numTrips];
            for (int i = 0; i < numTrips; i++) {
                assignmentChanges[i] = solver.makeIsDifferentCstVar(vehicleAssignments[i], initialAssignments[i]);
            }
            IntVar totalChanges = solver.makeSum(assignmentChanges).var();

            // Preference for pooling: Add soft constraints to prefer pooling trips rather than assigning them to empty vehicles
            IntVar[] poolingPenalty = new IntVar[numTrips];
            for (int i = 0; i < numTrips; i++) {
                IntVar penalty = solver.makeIntVar(0, VEHICLE_CAPACITY * VEHICLE_CAPACITY, "penalty_" + i);

                // Calculate the penalty based on the number of empty seats
                for (int vehicleId : tripVehicleMap.get(subTrips.get(i).getTripId()).stream().mapToInt(v -> Integer.parseInt(v.getId().toString())).toArray()) {
                    IntVar vehicleLoad = solver.makeIntVar(0, VEHICLE_CAPACITY, "vehicleLoad_" + i);
                    for (int j = 0; j < numTrips; j++) {
                        vehicleLoad = solver.makeSum(vehicleLoad, solver.makeIsEqualCstVar(vehicleAssignments[j], vehicleId)).var();
                    }
                    IntVar emptySeats = solver.makeDifference(VEHICLE_CAPACITY, vehicleLoad).var();
                    penalty = solver.makeSum(penalty, solver.makeProd(emptySeats, emptySeats)).var(); // Penalize more for more empty seats
                }
                poolingPenalty[i] = penalty;
            }
            IntVar totalPoolingPenalty = solver.makeSum(poolingPenalty).var();

            // Combine the objectives
            IntVar combinedObjective = solver.makeSum(totalChanges, totalPoolingPenalty).var();
            OptimizeVar objective = solver.makeMinimize(combinedObjective, 1);

            // Set up the search parameters with the number of workers (threads)
            // There is no direct way to set the number of threads in the CP solver in Java as in other APIs.
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
            // Suggest the JVM run garbage collection
            System.gc();
            System.runFinalization();
        }
    }

}