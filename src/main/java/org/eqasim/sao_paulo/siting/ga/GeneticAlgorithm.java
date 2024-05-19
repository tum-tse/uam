package org.eqasim.sao_paulo.siting.ga;

import net.bhl.matsim.uam.infrastructure.UAMStation;
import org.eqasim.sao_paulo.siting.Utils.*;

import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import net.bhl.matsim.uam.infrastructure.UAMVehicleType;
import org.matsim.contrib.dvrp.fleet.ImmutableDvrpVehicleSpecification;
import org.matsim.api.core.v01.Id;

public class GeneticAlgorithm {
    private static final int POP_SIZE = 100; // Population size
    private static final int MAX_GENERATIONS = 100; // Max number of generations
    private static final double MUTATION_RATE = 0.05; // Mutation rate
    private static final double CROSSOVER_RATE = 0.7; // Crossover rate
    private static final int TOURNAMENT_SIZE = 5; // Tournament size for selection
    private static final int CROSSOVER_DISABLE_AFTER = 100; // New field to control when to stop crossover
    private static final long SEED = 4711; // MATSim default Random Seed
    private static final Random rand = new Random(SEED);
    private static final int BUFFER_START_TIME = 3600*7; // Buffer start time for the first trip
    private static final int BUFFER_END_TIME = 3600*7+240; // Buffer end time for the last trip

    private static final double ALPHA = - 1.0; // Weight for changed flight distances
    private static final double BETA = - 0.5; // Weight for additional travel time
    private static final double BETA_NONE_POOLED_TRIP_EARLIER_DEPARTURE = - 0.1; //TODO: need to reconsider the value
    private static final double PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION = -1000;

    private static final int VEHICLE_CAPACITY = 4; // Vehicle capacity
    private static final double VEHICLE_CRUISE_SPEED = 180000.0 / 3600.0; // Vehicle cruise speed in m/s
    private static final double SEARCH_RADIUS_ORIGIN = 200000; // search radius for origin station
    private static final double SEARCH_RADIUS_DESTINATION = 200000; // search radius for destination station

    private static final int VALUE_FOR_NO_VEHICLE_AVAILABLE = -1; // For example, using -1 as an indicator of no vehicle available
    private static final double END_SERVICE_TIME_OF_THE_DAY = 3600*36; // End service time of the day
    private static int FIRST_UAM_VEHICLE_ID = 1;

    private static final int THRESHOLD_FOR_TRIPS_LONGER_THAN = 15000;
    private static final String THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING = String.valueOf(THRESHOLD_FOR_TRIPS_LONGER_THAN);
    private static int NUMBER_OF_TRIPS_LONGER_TAHN = 0;

    // Assuming these arrays are initialized elsewhere in your code:
    private static double[] flightDistances; // Distances for each trip
    private static double[][] accessTimesOriginal; // Original access times for each trip
    private static double[][] accessTimesUpdated; // Updated access times for each trip and vehicle
    private static double[][] egressTimesOriginal; // Original egress times for each trip
    private static double[][] egressTimesUpdated; // Updated egress times for each trip and vehicle
    private static double[][] waitingTimes; // Waiting times at the parking station for each trip and vehicle

    private static final PriorityQueue<SolutionFitnessPair> solutionsHeap = new PriorityQueue<>(Comparator.comparingDouble(SolutionFitnessPair::getFitness));
    private static List<UAMTrip> subTrips = null;
    private static final Map<Id<DvrpVehicle>, UAMVehicle> vehicles = new HashMap<>();
    private static final Map<Id<DvrpVehicle>, UAMStation> vehicleOriginStationMap = new HashMap<>();
    private static Map<Id<UAMStation>, UAMStation> stations = null;
    private static Map<Id<UAMStation>, List<UAMVehicle>> originStationVehicleMap = null;
    private static final Map<Id<DvrpVehicle>, UAMStation> vehicleDestinationStationMap = new HashMap<>();
    private static Map<String, Map<UAMVehicle, Integer>> tripVehicleMap = null;

    // Main method to run the GA
    public static void main(String[] args) throws IOException {
        // Load data
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();
        subTrips = extractSubTrips(dataLoader.getUamTrips());
        //vehicles = dataLoader.getVehicles();
        stations = dataLoader.getStations();
        originStationVehicleMap = saveStationVehicleNumber(subTrips);
        tripVehicleMap = findNearbyVehiclesToTrips(subTrips, originStationVehicleMap);

        // Initialize population and solutions heap in the first generation
        int[][] population = initializePopulation();
        if (solutionsHeap.isEmpty()) {
            for (int[] individual : population) {
                double fitness = calculateFitness(individual);
                solutionsHeap.add(new SolutionFitnessPair(individual, fitness));
            }
        }

        // GA iterations
        for (int gen = 1; gen < MAX_GENERATIONS; gen++) {
            resetVehicleCapacities(tripVehicleMap); // Reset the vehicle capacity at the beginning of each GA iteration since capacity of vehicles will be updated during each iteration
            population = evolvePopulation(population, gen);
            updateSolutionsHeap(population);
            System.out.println("Generation " + gen + ": Best fitness = " + findBestFitness(population));
        }

        // Output best solution details
        System.out.println("Best solution across iterations: " + Arrays.toString(solutionsHeap.peek().getSolution()));

        // Print the NUMBER_OF_TRIPS_LONGER_TAHN_1KM
        System.out.println("Threshold for trips longer than " + THRESHOLD_FOR_TRIPS_LONGER_THAN_STRING + ": " + NUMBER_OF_TRIPS_LONGER_TAHN);
    }
    // Find the best fitness in the current population
    private static double findBestFitness(int[][] population) {
        double bestFitness = Double.MIN_VALUE;
        for (int[] individual : population) {
            double fitness = calculateFitness(individual);
            if (fitness > bestFitness) {
                bestFitness = fitness;
            }
        }
        return bestFitness;
    }

    // New method to update the solutions heap
    private static void updateSolutionsHeap(int[][] population) {
        for (int[] individual : population) {
            double fitness = calculateFitness(individual);
            if (fitness > solutionsHeap.peek().getFitness()) {
                solutionsHeap.poll(); // Remove the worst solution
                solutionsHeap.add(new SolutionFitnessPair(individual, fitness)); // Add the new better solution
            }
        }
    }

    // SolutionFitnessPair class to hold individual solutions and their fitness
    private static class SolutionFitnessPair {
        private int[] solution;
        private double fitness;

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

    private static ArrayList<UAMTrip> extractSubTrips(List<UAMTrip> uamTrips) {
        // extract sub trips from uamTrips based on the departure time of trips falling between buffer start and end time
        return uamTrips.stream()
                .filter(trip -> trip.getDepartureTime() >= BUFFER_START_TIME && trip.getDepartureTime() < BUFFER_END_TIME)
                .collect(Collectors.toCollection(ArrayList::new));
    }

    private static Map<String, Map<UAMVehicle, Integer>> findNearbyVehiclesToTrips(List<UAMTrip> subTrips, Map<Id<UAMStation>, List<UAMVehicle>> originStationVehicleMap) {
        Map<String, Map<UAMVehicle, Integer>> tripVehicleMap = new HashMap<>();
        for (UAMTrip trip : subTrips) {
            for (UAMStation station : stations.values()) {
                if (trip.calculateAccessTeleportationDistance(station) <= SEARCH_RADIUS_ORIGIN) {
                    if (trip.calculateAccessTeleportationDistance(station)> THRESHOLD_FOR_TRIPS_LONGER_THAN){
                        NUMBER_OF_TRIPS_LONGER_TAHN++;
                    }
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
                    throw new IllegalArgumentException("trip.calculateTeleportationDistance(station) <= SEARCH_RADIUS_ORIGIN");
                }
            }
        }
        return tripVehicleMap;
    }
    private static void resetVehicleCapacities(Map<String, Map<UAMVehicle, Integer>> tripVehicleMap) {
        for (Map<UAMVehicle, Integer> vehicleMap : tripVehicleMap.values()) {
            vehicleMap.forEach((vehicle, capacity) -> {
                vehicleMap.put(vehicle, VEHICLE_CAPACITY); // Reset capacity to 4
            });
        }
    }

    // Initialize population with random assignments
    private static int[][] initializePopulation() {
        int[][] population = new int[GeneticAlgorithm.POP_SIZE][];
        for (int i = 0; i < GeneticAlgorithm.POP_SIZE; i++) {
            population[i] = generateIndividual();
        }
        return population;
    }

    private static void assignAvailableVehicle(int i, int[] individual) {
        Map<UAMVehicle, Integer> vehicleCapacityMap = tripVehicleMap.get(subTrips.get(i).getTripId());
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
            for (UAMVehicle vehicle: vehicleList){
                UAMStation destinationStation = vehicleDestinationStationMap.get(vehicle.getId());
                if (subTrips.get(i).calculateEgressTeleportationDistance(destinationStation) > SEARCH_RADIUS_DESTINATION){
                    vehicleList.remove(vehicle);
                }
            }
            if (vehicleList.isEmpty()){
                throw new IllegalArgumentException("No available vehicle for the trip, Please check the reason.");
            }

            //add access constraint
            int vehicleIndex = rand.nextInt(vehicleList.size());
            UAMVehicle selectedVehicle = vehicleList.get(vehicleIndex);
            Integer currentCapacity = vehicleCapacityMap.get(selectedVehicle);
            if (currentCapacity > 0) {
                individual[i] = Integer.parseInt(selectedVehicle.getId().toString());

                // Decrement capacity and explicitly update tripVehicleMap
                vehicleCapacityMap.put(selectedVehicle, currentCapacity - 1);
                tripVehicleMap.put(subTrips.get(i).getTripId(), vehicleCapacityMap);
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

    // Generate a random individual
    private static int[] generateIndividual() {
        int[] individual = new int[subTrips.size()];
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
        double bestFitness = Double.MIN_VALUE;

        for (int i = 0; i < TOURNAMENT_SIZE; i++) {
            int[] individual = pop[rand.nextInt(POP_SIZE)];
            double fitness = calculateFitness(individual);
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
        return individual;
    }

    // Calculate fitness for an individual
    private static double calculateFitness(int[] individual) {
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

        // Calculate fitness per vehicle
        for (Map.Entry<Integer, List<UAMTrip>> entry : vehicleAssignments.entrySet()) {
            List<UAMTrip> trips = entry.getValue();
            UAMStation originStationOfVehicle = vehicleOriginStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));
            UAMStation destinationStationOfVehicle = vehicleDestinationStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));

            // safety check
            if (trips.isEmpty()) continue;
            if (trips.size() == 1){
                UAMTrip trip = trips.get(0);
                fitness = getFitnessForNonPooledOrBaseTrip(trip, originStationOfVehicle, destinationStationOfVehicle, fitness);
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
/*                    double originalArrivalTimeForBaseTrip = baseTrip.getDepartureTime() + baseTrip.calculateTeleportationTime(originStationOfVehicle);
                    double additionalTravelTimeDueToAccessMatching = originalArrivalTimeForBaseTrip;
                    fitness += BETA * additionalTravelTimeDueToAccessMatching;*/
                    fitness = getFitnessForNonPooledOrBaseTrip(trip, originStationOfVehicle, destinationStationOfVehicle, fitness);
                    continue;
                }

                // calculate change in flight distance
                double flightDistanceChange = getFlightDistanceChange(trip, originStationOfVehicle, destinationStationOfVehicle);
                fitness += ALPHA * flightDistanceChange;
                // calculate saved flight distance
                double savedFlightDistance = trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation());
                fitness += ALPHA * (-1) * savedFlightDistance;
                // calculate change in flight time due to the change in flight distance
                double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
                fitness += BETA * flightTimeChange;
                // calculate additional travel time
                double originalArrivalTimeForThePooledTrip = trip.getDepartureTime() + trip.calculateAccessTeleportationTime(trip.getOriginStation());
                double additionalTravelTimeDueToAccessMatching = boardingTimeForAllTrips - originalArrivalTimeForThePooledTrip;
                if (additionalTravelTimeDueToAccessMatching < 0){
                    additionalTravelTimeDueToAccessMatching = 0;
                    //TODO: reconsider for "negative additional travel time" cases
                }
                fitness += BETA * additionalTravelTimeDueToAccessMatching;
                double additionalTravelTimeDueToEgressMatching = getAdditionalTravelTimeDueToEgressMatching(trip, destinationStationOfVehicle);
                fitness += BETA * additionalTravelTimeDueToEgressMatching;
            }
            //add penalty for the case when vehicle capacity is violated
            if(trips.size()>VEHICLE_CAPACITY){
                fitness += PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION * (trips.size()-VEHICLE_CAPACITY);
            }
        }

        return fitness;
    }
    private static double getFitnessForNonPooledOrBaseTrip(UAMTrip trip, UAMStation originStationOfVehicle, UAMStation destinationStationOfVehicle, double fitness) {
        // calculate change in flight distance
        double flightDistanceChange = getFlightDistanceChange(trip, originStationOfVehicle, destinationStationOfVehicle);
        fitness += ALPHA * flightDistanceChange;
        // calculate change in flight time due to the change in flight distance
        double flightTimeChange = flightDistanceChange / VEHICLE_CRUISE_SPEED;
        fitness += BETA * flightTimeChange;
        // calculate additional travel time due to access matching
        double additionalTravelTimeDueToAccessMatching = trip.calculateAccessTeleportationTime(originStationOfVehicle) - trip.calculateAccessTeleportationTime(trip.getOriginStation());
        if(additionalTravelTimeDueToAccessMatching > 0) {
            fitness += BETA * additionalTravelTimeDueToAccessMatching;
        } else {
            fitness += BETA_NONE_POOLED_TRIP_EARLIER_DEPARTURE * additionalTravelTimeDueToAccessMatching;
        }
        // calculate additional travel time due to egress matching
        double additionalTravelTimeDueToEgressMatching = getAdditionalTravelTimeDueToEgressMatching(trip, destinationStationOfVehicle);
        fitness += BETA * additionalTravelTimeDueToEgressMatching;
        return fitness;
    }
    private static double getFlightDistanceChange(UAMTrip trip, UAMStation originStationOfVehicle, UAMStation destinationStationOfVehicle) {
        return trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle) - trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation());
    }
    private static double getAdditionalTravelTimeDueToEgressMatching(UAMTrip trip, UAMStation destinationStationOfVehicle) {
        return trip.calculateEgressTeleportationTime(destinationStationOfVehicle) - trip.calculateEgressTeleportationTime(trip.getDestinationStation());
    }


    private static Map<Id<UAMStation>, List<UAMVehicle>> saveStationVehicleNumber(List<UAMTrip> subTrips) {
        Map<Id<UAMStation>, List<UAMVehicle>> oringinStationVehicleMap = new HashMap<>();

/*        UAMVehicleType vehicleType = new UAMVehicleType(id, capacity, range, horizontalSpeed, verticalSpeed,
                boardingTime, deboardingTime, turnAroundTime, energyConsumptionVertical, energyConsumptionHorizontal,
                maximumCharge);*/
        final Map<Id<UAMVehicleType>, UAMVehicleType> vehicleTypes = new HashMap<>();
        Id<UAMVehicleType> vehicleTypeId = Id.create("poolingVehicle", UAMVehicleType.class);
        UAMVehicleType vehicleType = new UAMVehicleType(vehicleTypeId, 0, 0, 0, 0,
                0, 0, 0);
        vehicleTypes.put(vehicleTypeId, vehicleType);

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
            UAMVehicle vehicle = createVehicle(subTrip.getOriginStation(), vehicleTypes.get(vehicleTypeId));

            // Get the station ID
            Id<UAMStation> stationId = subTrip.getOriginStation().getId();
            // Check if there is already a list for this station ID, if not, create one
            List<UAMVehicle> vehiclesAtStation = oringinStationVehicleMap.computeIfAbsent(stationId, k -> new ArrayList<>());
            // Add the new vehicle to the list
            vehiclesAtStation.add(vehicle);
            vehicles.put(vehicle.getId(), vehicle);
            vehicleOriginStationMap.put(vehicle.getId(), subTrip.getOriginStation());
            vehicleDestinationStationMap.put(vehicle.getId(), subTrip.getDestinationStation());
            oringinStationVehicleMap.put(stationId, vehiclesAtStation);
        }
        return oringinStationVehicleMap;
    }
    // vehicle creator function
    private static UAMVehicle createVehicle(UAMStation uamStation, UAMVehicleType vehicleType) {
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

}
//TODO: need to check if the best solution violates all hard constraints or not
//TODO: Need use the best solution starting from the crossover_disable_after iteration to generate new solutions for remaining iterations
//TODO: Need to print the performance indicators for the best solution of each iteration. Especially, we want to observe the indicators for the trips has the extremely bad performance