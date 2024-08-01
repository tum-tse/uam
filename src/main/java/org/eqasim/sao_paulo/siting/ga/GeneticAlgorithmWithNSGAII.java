package org.eqasim.sao_paulo.siting.ga;

import org.apache.commons.lang3.tuple.Pair;
import org.eqasim.sao_paulo.siting.Utils.*;
import org.matsim.api.core.v01.Id;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import org.matsim.contrib.dvrp.fleet.ImmutableDvrpVehicleSpecification;
import net.bhl.matsim.uam.infrastructure.UAMStation;
import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import net.bhl.matsim.uam.infrastructure.UAMVehicleType;
import org.apache.log4j.Logger;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.crossover.impl.PMXCrossover;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.mutation.impl.PermutationSwapMutation;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.BinaryTournamentSelection;
import org.uma.jmetal.problem.permutationproblem.PermutationProblem;
import org.uma.jmetal.solution.permutationsolution.PermutationSolution;
import org.uma.jmetal.solution.permutationsolution.impl.IntegerPermutationSolution;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class GeneticAlgorithmWithNSGAII {
    private static final Logger log = Logger.getLogger(GeneticAlgorithmWithNSGAII.class);

    // NSGA-II parameters
    private static final int MAX_EVALUATIONS = 100000;
    private static final int POPULATION_SIZE = 100;
    private static final double CROSSOVER_PROBABILITY = 0.9;
    private static final double MUTATION_PROBABILITY = 1.0 / 50;
    private static final int VALUE_FOR_NO_VEHICLE_AVAILABLE = -1;

    // UAM problem parameters
    private static final long SEED = 4711;
    private static final Random rand = new Random(SEED);

    private static final double ALPHA = -1;
    private static final double BETA = -1;
    private static final double PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION = -10000;

    private static final int VEHICLE_CAPACITY = 4;

    private static final double SEARCH_RADIUS_ORIGIN = 1500; // search radius for origin station
    private static final double SEARCH_RADIUS_DESTINATION = 1500; // search radius for destination station

    private static List<UAMTrip> subTrips;
    private static Map<Id<UAMStation>, UAMStation> stations;
    private static Map<String, List<UAMVehicle>> tripVehicleMap;
    private static Map<Id<DvrpVehicle>, UAMStation> vehicleOriginStationMap = new HashMap<>();
    private static Map<Id<DvrpVehicle>, UAMStation> vehicleDestinationStationMap = new HashMap<>();
    private static Map<Id<UAMStation>, List<UAMVehicle>> originStationVehicleMap = new HashMap<>();
    private static int FIRST_UAM_VEHICLE_ID = 1;

    public static void main(String[] args) throws Exception {
        // Load data
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();

        String filePath = "/home/tumtse/Documents/haowu/uam/uam/scenarios/1-percent/sao_paulo_population2trips.csv";
        subTrips = readTripsFromCsv(filePath);
        subTrips = subTrips.stream()
                .filter(trip -> rand.nextDouble() < 0.01)
                .collect(Collectors.toCollection(ArrayList::new));

        log.info("The number of UAM trips: " + subTrips.size());

        stations = dataLoader.getStations();

        for (UAMTrip uamTrip : subTrips) {
            uamTrip.setOriginStation(findNearestStation(uamTrip, stations, true));
            uamTrip.setDestinationStation(findNearestStation(uamTrip, stations, false));
        }
        saveStationVehicleNumber(subTrips);
        tripVehicleMap = findNearbyVehiclesToTrips(subTrips);

        // Define the problem
        PermutationProblem<PermutationSolution<Integer>> problem = new UAMPermutationProblem(subTrips.size(), 2);

        // Define the crossover operator
        CrossoverOperator<PermutationSolution<Integer>> crossover = new PMXCrossover(CROSSOVER_PROBABILITY);

        // Define the mutation operator
        MutationOperator<PermutationSolution<Integer>> mutation = new CustomPermutationSwapMutation(MUTATION_PROBABILITY);

        // Define the selection operator
        SelectionOperator<List<PermutationSolution<Integer>>, PermutationSolution<Integer>> selection = new BinaryTournamentSelection<>();

        // Create the NSGA-II algorithm
        NSGAIIBuilder<PermutationSolution<Integer>> builder = new NSGAIIBuilder<>(problem, crossover, mutation, POPULATION_SIZE);
        builder.setSelectionOperator(selection);
        builder.setSolutionListEvaluator(new SequentialSolutionListEvaluator<>());
        builder.setMaxEvaluations(MAX_EVALUATIONS);
        builder.setOffspringPopulationSize(POPULATION_SIZE);

        var algorithm = builder.build();

        // Run the algorithm
        algorithm.run();

        // Get the result
        List<PermutationSolution<Integer>> result = algorithm.getResult();

        // Print the Pareto front
        for (PermutationSolution<Integer> solution : result) {
            System.out.println("Solution: " + solution.getObjective(0) + ", " + solution.getObjective(1));
        }
    }

    public static class UAMPermutationProblem implements PermutationProblem<PermutationSolution<Integer>> {
        private final int numberOfVariables;
        private final int numberOfObjectives;
        private final int numberOfConstraints; // Added this line

        public UAMPermutationProblem(int numberOfVariables, int numberOfObjectives) {
            this.numberOfVariables = numberOfVariables;
            this.numberOfObjectives = numberOfObjectives;
            this.numberOfConstraints = 1; // Set the number of constraints here
        }

        @Override
        public int getNumberOfVariables() {
            return numberOfVariables;
        }

        @Override
        public int getNumberOfObjectives() {
            return numberOfObjectives;
        }

        @Override
        public int getNumberOfConstraints() {
            return numberOfConstraints; // Added this line
        }

        @Override
        public String getName() {
            return "UAMPermutationProblem";
        }

        @Override
        public PermutationSolution<Integer> createSolution() {
            PermutationSolution<Integer> solution = new IntegerPermutationSolution(numberOfVariables, numberOfObjectives); // Initialize with number of constraints
            for (int i = 0; i < numberOfVariables; i++) {
                assignAvailableVehicle(i, solution);
            }
            return solution;
        }

        @Override
        public void evaluate(PermutationSolution<Integer> solution) {
            int[] individual = new int[getNumberOfVariables()];
            for (int i = 0; i < getNumberOfVariables(); i++) {
                individual[i] = solution.getVariable(i);
            }

            double[] objectives = calculateObjectives(individual);
            solution.setObjective(0, objectives[0]);
            solution.setObjective(1, objectives[1]);

            double penalty = computePenalty(individual);
            if (solution.getNumberOfConstraints() > 0) {
                solution.setConstraint(0, penalty); // Ensure constraints are set properly
            }
        }

        // Rest of the UAMPermutationProblem class remains unchanged

        private double[] calculateObjectives(int[] individual) {
            double savedFlightDistance = computeSavedFlightDistance(individual);
            double additionalTravelTime = computeAdditionalTravelTime(individual);

            return new double[]{savedFlightDistance, additionalTravelTime};
        }

        private double computeSavedFlightDistance(int[] individual) {
            double totalSavedDistance = 0.0;

            Map<Integer, List<UAMTrip>> vehicleAssignments = new HashMap<>();
            for (int i = 0; i < individual.length; i++) {
                int vehicleId = individual[i];
                vehicleAssignments.putIfAbsent(vehicleId, new ArrayList<>());
                vehicleAssignments.get(vehicleId).add(subTrips.get(i));
            }

            for (Map.Entry<Integer, List<UAMTrip>> entry : vehicleAssignments.entrySet()) {
                List<UAMTrip> trips = entry.getValue();
                UAMStation originStationOfVehicle = vehicleOriginStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));
                UAMStation destinationStationOfVehicle = vehicleDestinationStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));

                if (trips.isEmpty()) continue;

                for (UAMTrip trip : trips) {
                    double savedDistance = trip.calculateFlightDistance(trip.getOriginStation(), trip.getDestinationStation())
                            - trip.calculateFlightDistance(originStationOfVehicle, destinationStationOfVehicle);
                    totalSavedDistance += savedDistance;
                }
            }

            return totalSavedDistance;
        }

        private double computeAdditionalTravelTime(int[] individual) {
            double totalAdditionalTravelTime = 0.0;

            Map<Integer, List<UAMTrip>> vehicleAssignments = new HashMap<>();
            for (int i = 0; i < individual.length; i++) {
                int vehicleId = individual[i];
                vehicleAssignments.putIfAbsent(vehicleId, new ArrayList<>());
                vehicleAssignments.get(vehicleId).add(subTrips.get(i));
            }

            for (Map.Entry<Integer, List<UAMTrip>> entry : vehicleAssignments.entrySet()) {
                List<UAMTrip> trips = entry.getValue();
                UAMStation originStationOfVehicle = vehicleOriginStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));
                UAMStation destinationStationOfVehicle = vehicleDestinationStationMap.get(Id.create(entry.getKey().toString(), DvrpVehicle.class));

                if (trips.isEmpty()) continue;

                for (UAMTrip trip : trips) {
                    double additionalTime = trip.calculateAccessTeleportationTime(originStationOfVehicle) + trip.calculateEgressTeleportationTime(destinationStationOfVehicle)
                            - trip.calculateAccessTeleportationTime(trip.getOriginStation()) - trip.calculateEgressTeleportationTime(trip.getDestinationStation());
                    totalAdditionalTravelTime += additionalTime;
                }
            }

            return totalAdditionalTravelTime;
        }

        private double computePenalty(int[] individual) {
            double penalty = 0.0;
            Map<Integer, Integer> vehicleLoad = new HashMap<>();

            for (int vehicleId : individual) {
                vehicleLoad.put(vehicleId, vehicleLoad.getOrDefault(vehicleId, 0) + 1);
            }

            for (int load : vehicleLoad.values()) {
                if (load > VEHICLE_CAPACITY) {
                    penalty += PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION * (load - VEHICLE_CAPACITY);
                }
            }

            return penalty;
        }

        @Override
        public int getLength() {
            return numberOfVariables;
        }
    }

    public static class CustomPermutationSwapMutation implements MutationOperator<PermutationSolution<Integer>> {
        private final double mutationProbability;

        public CustomPermutationSwapMutation(double mutationProbability) {
            this.mutationProbability = mutationProbability;
        }

        @Override
        public PermutationSolution<Integer> execute(PermutationSolution<Integer> solution) {
            if (solution == null) {
                throw new IllegalArgumentException("Null parameter");
            }

            doMutation(mutationProbability, solution);
            return solution;
        }

        private void doMutation(double probability, PermutationSolution<Integer> solution) {
            for (int i = 0; i < solution.getNumberOfVariables(); i++) {
                if (rand.nextDouble() <= probability) {
                    assignAvailableVehicle(i, solution);
                }
            }
        }

        @Override
        public double getMutationProbability() {
            return mutationProbability;
        }
    }

    private static void assignAvailableVehicle(int i, Object individual) {
        UAMTrip trip = subTrips.get(i);
        List<UAMVehicle> vehicleList = tripVehicleMap.get(trip.getTripId());

        synchronized (tripVehicleMap) {
            // Add a new vehicle for the trip when there is no available vehicle
            if (vehicleList == null || vehicleList.isEmpty()) {
                if (vehicleList == null) {
                    vehicleList = new ArrayList<>();
                }
                UAMVehicle vehicle = feedDataForVehicleCreation(trip, false);
                vehicleList.add(vehicle);
                tripVehicleMap.put(trip.getTripId(), vehicleList);
            }
        }

        if (!vehicleList.isEmpty()) {
            int vehicleIndex = rand.nextInt(vehicleList.size());
            UAMVehicle selectedVehicle = vehicleList.get(vehicleIndex);
            if (individual instanceof int[]) {
                ((int[]) individual)[i] = Integer.parseInt(selectedVehicle.getId().toString());
            } else if (individual instanceof PermutationSolution) {
                ((PermutationSolution<Integer>) individual).setVariable(i, Integer.parseInt(selectedVehicle.getId().toString()));
            }
        } else {
            if (individual instanceof int[]) {
                ((int[]) individual)[i] = VALUE_FOR_NO_VEHICLE_AVAILABLE;
            } else if (individual instanceof PermutationSolution) {
                ((PermutationSolution<Integer>) individual).setVariable(i, VALUE_FOR_NO_VEHICLE_AVAILABLE);
            }
            throw new IllegalArgumentException("Need to handle the case when there is no available vehicle for the trip.");
        }
    }

    // Other helper methods for UAM problem

    private static UAMStation findNearestStation(UAMTrip trip, Map<Id<UAMStation>, UAMStation> stations, boolean accessLeg) {
        UAMStation nearestStation = null;
        double shortestDistance = Double.MAX_VALUE;
        for (UAMStation station : stations.values()) {
            double distance = accessLeg ? trip.calculateAccessTeleportationDistance(station) : trip.calculateEgressTeleportationDistance(station);
            if (distance < shortestDistance) {
                nearestStation = station;
                shortestDistance = distance;
            }
        }
        return nearestStation;
    }

    private static Map<String, List<UAMVehicle>> findNearbyVehiclesToTrips(List<UAMTrip> subTrips) {
        Map<String, List<UAMVehicle>> tripVehicleMap = new HashMap<>();
        for (UAMTrip trip : subTrips) {
            for (UAMStation station : stations.values()) {
                if (trip.calculateAccessTeleportationDistance(station) <= SEARCH_RADIUS_ORIGIN) {
                    List<UAMVehicle> vehicles = originStationVehicleMap.get(station.getId());
                    if (vehicles == null) {
                        continue;
                    }
                    List<UAMVehicle> existingVehicles = tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>());

                    // add egress constraint
                    vehicles = vehicles.stream()
                            .filter(vehicle -> trip.calculateEgressTeleportationDistance(vehicleDestinationStationMap.get(vehicle.getId())) <= SEARCH_RADIUS_DESTINATION)
                            .collect(Collectors.toCollection(ArrayList::new));

                    existingVehicles.addAll(vehicles);

                    tripVehicleMap.put(trip.getTripId(), existingVehicles);
                }
            }
        }
        return tripVehicleMap;
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
    private static UAMVehicle createVehicle(UAMStation uamStation) {
        Id<UAMVehicleType> vehicleTypeId = Id.create("poolingVehicle", UAMVehicleType.class);
        UAMVehicleType vehicleType = new UAMVehicleType(vehicleTypeId, 0, 0, 0, 0, 0, 0, 0);

        ImmutableDvrpVehicleSpecification.Builder builder = ImmutableDvrpVehicleSpecification.newBuilder();
        builder.id(Id.create(String.valueOf(FIRST_UAM_VEHICLE_ID++), DvrpVehicle.class));
        builder.startLinkId(uamStation.getLocationLink().getId());
        builder.capacity(VEHICLE_CAPACITY);
        builder.serviceBeginTime(3600 * 7);
        builder.serviceEndTime(3600 * 36);
        ImmutableDvrpVehicleSpecification vehicleSpecification = builder.build();

        return new UAMVehicle(vehicleSpecification,
                uamStation.getLocationLink(), uamStation.getId(), vehicleType);
    }

    public static List<UAMTrip> readTripsFromCsv(String filePath) {
        List<UAMTrip> trips = new ArrayList<>();

        try (Stream<String> lines = Files.lines(Paths.get(filePath))) {
            trips = lines
                    .skip(1)
                    .map(GeneticAlgorithmWithNSGAII::parseTrip)
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

        double flightDistance = 0.0;
        UAMStation origStation = null;
        UAMStation destStation = null;
        String income = "0";

        return new UAMTrip(tripId, originX, originY, destX, destY, departureTime, flightDistance, origStation, destStation, purpose, income);
    }
}