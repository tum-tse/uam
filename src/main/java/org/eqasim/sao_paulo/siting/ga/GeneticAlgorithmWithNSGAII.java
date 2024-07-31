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
import org.uma.jmetal.operator.crossover.impl.IntegerSBXCrossover;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.mutation.impl.IntegerPolynomialMutation;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.integersolution.impl.DefaultIntegerSolution;
import org.uma.jmetal.solution.integersolution.IntegerSolution;
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
    private static final double DISTRIBUTION_INDEX_FOR_CROSSOVER = 20.0;
    private static final double DISTRIBUTION_INDEX_FOR_MUTATION = 20.0;

    // UAM problem parameters
    private static final long SEED = 4711;
    private static final Random rand = new Random(SEED);

    private static final double ALPHA = -1;
    private static final double BETA = -1;
    private static final double PENALTY_FOR_VEHICLE_CAPACITY_VIOLATION = -10000;

    private static final int VEHICLE_CAPACITY = 4;

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
        Problem<IntegerSolution> problem = new UAMProblem(
                subTrips.stream().map(trip -> Pair.of(0, tripVehicleMap.get(trip.getTripId()).size() - 1)).collect(Collectors.toList()),
                2,
                1
        );

        // Define the crossover operator
        CrossoverOperator<IntegerSolution> crossover = new IntegerSBXCrossover(CROSSOVER_PROBABILITY, DISTRIBUTION_INDEX_FOR_CROSSOVER);

        // Define the mutation operator
        MutationOperator<IntegerSolution> mutation = new IntegerPolynomialMutation(MUTATION_PROBABILITY, DISTRIBUTION_INDEX_FOR_MUTATION);

        // Define the selection operator
        SelectionOperator<List<IntegerSolution>, IntegerSolution> selection = new BinaryTournamentSelection<>();

        // Create the NSGA-II algorithm
        NSGAIIBuilder<IntegerSolution> builder = new NSGAIIBuilder<>(problem, crossover, mutation, POPULATION_SIZE);
        builder.setSelectionOperator(selection);
        builder.setSolutionListEvaluator(new SequentialSolutionListEvaluator<>());
        builder.setMaxEvaluations(MAX_EVALUATIONS);
        builder.setOffspringPopulationSize(POPULATION_SIZE); //.setPopulationSize(POPULATION_SIZE);

        var algorithm = builder.build();

        // Run the algorithm
        algorithm.run();

        // Get the result
        List<IntegerSolution> result = algorithm.getResult();

        // Print the Pareto front
        for (IntegerSolution solution : result) {
            System.out.println("Solution: " + solution.getObjective(0) + ", " + solution.getObjective(1));
        }
    }

    public static class UAMProblem extends DefaultIntegerSolution implements Problem<IntegerSolution> {
        /**
         * Constructor
         *
         * @param bounds
         * @param numberOfObjectives
         * @param numberOfConstraints
         */
        public UAMProblem(List<Pair<Integer, Integer>> bounds, int numberOfObjectives, int numberOfConstraints) {
            super(bounds, numberOfObjectives, numberOfConstraints);
        }

        @Override
        public int getNumberOfVariables() {
            return subTrips.size();
        }

        @Override
        public int getNumberOfObjectives() {
            return 2;
        }

        @Override
        public int getNumberOfConstraints() {
            return 1;
        }

        @Override
        public String getName() {
            return "UAM Problem";
        }

        @Override
        public IntegerSolution createSolution() {
            IntegerSolution solution = new DefaultIntegerSolution(this);
            for (int i = 0; i < getNumberOfVariables(); i++) {
                solution.setVariable(i, rand.nextInt(50));
            }
            return solution;
        }

        @Override
        public void evaluate(IntegerSolution solution) {
            int[] individual = new int[getNumberOfVariables()];
            for (int i = 0; i < getNumberOfVariables(); i++) {
                individual[i] = solution.getVariable(i);
            }

            double[] objectives = calculateObjectives(individual);
            solution.setObjective(0, objectives[0]);
            solution.setObjective(1, objectives[1]);
        }

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
                if (trip.calculateAccessTeleportationDistance(station) <= 1500) {
                    List<UAMVehicle> vehicles = originStationVehicleMap.get(station.getId());
                    List<UAMVehicle> existingVehicles = tripVehicleMap.getOrDefault(trip.getTripId(), new ArrayList<>());

                    vehicles = vehicles.stream()
                            .filter(vehicle -> trip.calculateEgressTeleportationDistance(vehicleDestinationStationMap.get(vehicle.getId())) <= 1500)
                            .collect(Collectors.toCollection(ArrayList::new));

                    existingVehicles.addAll(vehicles);
                    tripVehicleMap.put(trip.getTripId(), existingVehicles);
                }
            }
        }
        return tripVehicleMap;
    }

    private static void saveStationVehicleNumber(List<UAMTrip> subTrips) {
        for (UAMTrip subTrip : subTrips) {
            UAMStation nearestOriginStation = findNearestStation(subTrip, stations, true);
            UAMStation nearestDestinationStation = findNearestStation(subTrip, stations, false);

            UAMVehicle vehicle = createVehicle(nearestOriginStation);
            vehicleOriginStationMap.put(vehicle.getId(), nearestOriginStation);
            vehicleDestinationStationMap.put(vehicle.getId(), nearestDestinationStation);
        }
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
