package org.eqasim.sao_paulo.siting.ga;

import net.bhl.matsim.uam.infrastructure.UAMStation;
import org.eqasim.sao_paulo.siting.Utils.*;

import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import org.matsim.contrib.dvrp.fleet.FleetSpecificationImpl;
import net.bhl.matsim.uam.infrastructure.UAMVehicleType;
import org.matsim.contrib.dvrp.fleet.ImmutableDvrpVehicleSpecification;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.Id;

public class GeneticAlgorithm {
    private static final int POP_SIZE = 100; // Population size
    private static final int MAX_GENERATIONS = 100; // Max number of generations
    private static final double MUTATION_RATE = 0.05; // Mutation rate
    private static final double CROSSOVER_RATE = 0.7; // Crossover rate
    private static final int TOURNAMENT_SIZE = 5; // Tournament size for selection
    private static final long SEED = 4711; // MATSim default Random Seed
    private static final Random rand = new Random(SEED);
    private static final int BUFFER_START_TIME = 3600*7; // Buffer start time for the first trip
    private static final int BUFFER_END_TIME = 3600*7+240; // Buffer end time for the last trip

    private static final double ALPHA = 1.0; // Weight for saved flight distances
    private static final double BETA = 0.5; // Weight for additional travel time

    private static final int VEHICLE_CAPACITY = 4; // Vehicle capacity
    private static final double SEARCH_RADIUS_ORIGIN = 1000; // search radius for origin station
    private static final double SEARCH_RADIUS_DESTINATION = 1000; // search radius for destination station

    // Assuming these arrays are initialized elsewhere in your code:
    private static double[] flightDistances; // Distances for each trip
    private static double[][] accessTimesOriginal; // Original access times for each trip
    private static double[][] accessTimesUpdated; // Updated access times for each trip and vehicle
    private static double[][] egressTimesOriginal; // Original egress times for each trip
    private static double[][] egressTimesUpdated; // Updated egress times for each trip and vehicle
    private static double[][] waitingTimes; // Waiting times at the parking station for each trip and vehicle

    private static List<UAMTrip> subTrips = null;
    //private static Map<Id<DvrpVehicle>, UAMVehicle> vehicles;
    private static Map<Id<UAMStation>, UAMStation> stations = null;
    private static Map<Id<UAMStation>, List<UAMVehicle>> stationVehicleMap = null;
    private static Map<String, List<UAMVehicle>> tripVehicleMap = null;

    // Main method to run the GA
    public static void main(String[] args) throws IOException {
        // Load data
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();
        subTrips = extractSubTrips(dataLoader.getUamTrips());
        //vehicles = dataLoader.getVehicles();
        stations = dataLoader.getStations();
        stationVehicleMap = saveStationVehicleNumber(subTrips);
        tripVehicleMap = findNearbyVehiclesToTrips(subTrips, stationVehicleMap);

        // GA
        int[][] population = initializePopulation();
        for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
            population = evolvePopulation(population);
            System.out.println("Generation " + gen + ": Best fitness = " + findBestFitness(population));
        }
    }

    private static List<UAMTrip> extractSubTrips(List<UAMTrip> uamTrips) {
        // extract sub trips from uamTrips based on the departure time of trips falling between buffer start and end time
        return uamTrips.stream()
                .filter(trip -> trip.getDepartureTime() >= BUFFER_START_TIME && trip.getDepartureTime() < BUFFER_END_TIME)
                .collect(Collectors.toList());
    }

    private static Map<String, List<UAMVehicle>> findNearbyVehiclesToTrips(List<UAMTrip> subTrips, Map<Id<UAMStation>, List<UAMVehicle>> stationVehicleMap) {
        Map<String, List<UAMVehicle>> tripVehicleMap = new HashMap<>();
        for (UAMTrip trip: subTrips) {
            for (UAMStation station: stations.values()) {
                if (trip.calculateTeleportationDistance(station) <= SEARCH_RADIUS_ORIGIN) {
                    List<UAMVehicle> vehicles = stationVehicleMap.get(station.getId());
                    List<UAMVehicle> existingVehicles = tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>());
                    existingVehicles.addAll(vehicles);
                    tripVehicleMap.put(trip.getTripId(), existingVehicles);
                }
            }
        }
        return tripVehicleMap;
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
        for (int i = 0; i < individual.length; i++) {
            List<UAMVehicle> vehicleList = tripVehicleMap.get(subTrips.get(i).getTripId()); // Retrieve the vehicle List using the trip ID as a key
            int vehicleIndex = rand.nextInt(vehicleList.size());
            individual[i] = Integer.parseInt(vehicleList.get(vehicleIndex).getId().toString());
        }
        return individual;
    }

    //TODO: continue from here
    // Evolve population
    private static int[][] evolvePopulation(int[][] pop) {
        int[][] newPop = new int[POP_SIZE][];
        for (int i = 0; i < POP_SIZE; i++) {
            int[] parent1 = select(pop);
            int[] parent2 = select(pop);
            int[] child = crossover(parent1, parent2);
            newPop[i] = mutate(child);
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
        }

        return Arrays.copyOf(best, best.length); // Return a copy of the best individual
    }

    // Crossover - Single point crossover
    private static int[] crossover(int[] parent1, int[] parent2) {
        int[] child = new int[parent1.length];
        if (rand.nextDouble() < CROSSOVER_RATE) {
            int crossoverPoint = rand.nextInt(parent1.length);
            for (int i = 0; i < crossoverPoint; i++) {
                child[i] = parent1[i];
            }
            for (int i = crossoverPoint; i < parent2.length; i++) {
                child[i] = parent2[i];
            }
        } else {
            return rand.nextBoolean() ? parent1 : parent2;
        }
        return child;
    }

    // Mutation - Randomly change vehicle assignment
    private static int[] mutate(int[] individual) {
        for (int i = 0; i < individual.length; i++) {
            if (rand.nextDouble() < MUTATION_RATE) {
                individual[i] = rand.nextInt(NUMBER_OF_VEHICLES);
            }
        }
        return individual;
    }

    // Calculate fitness for an individual
    private static double calculateFitness(int[] individual) {
        double fitness = 0;
        int trips = individual.length; // Number of trips

        for (int i = 0; i < trips; i++) {
            int v = individual[i]; // Vehicle assigned to trip i
            double savedFlightDistance = flightDistances[i];
            double additionalTravelTime = accessTimesUpdated[i][v] - accessTimesOriginal[i][v]
                    + egressTimesUpdated[i][v] - egressTimesOriginal[i][v]
                    + waitingTimes[i][v];

            fitness += (ALPHA * savedFlightDistance - BETA * additionalTravelTime);
        }

        return fitness;
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




    private static Map<Id<UAMStation>, List<UAMVehicle>> saveStationVehicleNumber(List<UAMTrip> subTrips) {
        Map<Id<UAMStation>, List<UAMVehicle>> stationVehicleMap = new HashMap<>();

        // initialize the map with the station and vehicle instances
        int intId = 1;

/*        UAMVehicleType vehicleType = new UAMVehicleType(id, capacity, range, horizontalSpeed, verticalSpeed,
                boardingTime, deboardingTime, turnAroundTime, energyConsumptionVertical, energyConsumptionHorizontal,
                maximumCharge);*/
        final Map<Id<UAMVehicleType>, UAMVehicleType> vehicleTypes = new HashMap<>();
        Id<UAMVehicleType> vehicleTypeId = Id.create("poolingVehicle", UAMVehicleType.class);
        UAMVehicleType vehicleType = new UAMVehicleType(vehicleTypeId, 0, 0, 0, 0,
                0, 0, 0);
        vehicleTypes.put(vehicleTypeId, vehicleType);

        // save the station's vehicle number for the current time based on the UAMTrips' origin station
        for (UAMTrip subTrip : subTrips) {
            // Create a builder instance
            ImmutableDvrpVehicleSpecification.Builder builder = ImmutableDvrpVehicleSpecification.newBuilder();
            // Set the properties of the vehicle
            builder.id(Id.create(String.valueOf(intId++), DvrpVehicle.class));
            builder.startLinkId(subTrip.getOriginStation().getLocationLink().getId());
            builder.capacity(VEHICLE_CAPACITY);
            builder.serviceBeginTime(BUFFER_START_TIME);
            builder.serviceEndTime(3600);
            // Build the vehicle specification
            ImmutableDvrpVehicleSpecification vehicleSpecification = builder.build();

            UAMVehicle vehicle = new UAMVehicle(vehicleSpecification,
                    subTrip.getOriginStation().getLocationLink(), subTrip.getOriginStation().getId(), vehicleTypes.get(vehicleTypeId));

            // Get the station ID
            Id<UAMStation> stationId = subTrip.getOriginStation().getId();
            // Check if there is already a list for this station ID, if not, create one
            List<UAMVehicle> vehiclesAtStation = stationVehicleMap.computeIfAbsent(stationId, k -> new ArrayList<>());
            // Add the new vehicle to the list
            vehiclesAtStation.add(vehicle);
        }
        return stationVehicleMap;
    }

}
