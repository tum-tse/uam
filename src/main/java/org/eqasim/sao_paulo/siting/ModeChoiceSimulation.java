package org.eqasim.sao_paulo.siting;

import java.io.*;
import java.util.*;

public class ModeChoiceSimulation {
    private static final int SCENARIOS_COUNT = 100;
    private static final long SEED = 4711; // MATSim default Random Seed

    public static void main(String[] args) throws IOException {
        List<Trip> trips = readTrips("scenarios/1-percent/UpdatedFinalTrips.csv");
        Random random = new Random(SEED); // Set the seed here
        List<List<ModeChoiceScenario>> allScenarios = generateScenarios(trips, SCENARIOS_COUNT, random);
        saveScenarios(allScenarios, "src/main/java/org/eqasim/sao_paulo/siting/mode_choice_scenarios.csv");
    }

    private static List<Trip> readTrips(String filename) throws IOException {
        List<Trip> trips = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line = reader.readLine(); // Skip header
            while ((line = reader.readLine()) != null) {
                String[] fields = line.split(",");
                Trip trip = new Trip(fields[0], Double.parseDouble(fields[15]), Double.parseDouble(fields[16]), Double.parseDouble(fields[17]));
                trips.add(trip);
            }
        }
        return trips;
    }

    private static List<List<ModeChoiceScenario>> generateScenarios(List<Trip> trips, int numScenarios, Random random) {
        List<List<ModeChoiceScenario>> allScenarios = new ArrayList<>();
        for (Trip trip : trips) {
            List<ModeChoiceScenario> tripScenarios = new ArrayList<>();
            for (int i = 1; i <= numScenarios; i++) { // Scenario index starts at 1
                String modeChoice = predictModeChoice(trip, random);
                tripScenarios.add(new ModeChoiceScenario(trip.tripId, modeChoice));
            }
            allScenarios.add(tripScenarios);
        }
        return allScenarios;
    }

    private static String predictModeChoice(Trip trip, Random random) {
        double totalUtility = trip.carUtility + trip.ptUtility + trip.uamUtility;
        double carProbability = trip.carUtility / totalUtility;
        double ptProbability = trip.ptUtility / totalUtility;
        double randomValue = random.nextDouble();

        if (randomValue < carProbability) {
            return "Car";
        } else if (randomValue < carProbability + ptProbability) {
            return "Public Transport";
        } else {
            return "UAM";
        }
    }

    private static void saveScenarios(List<List<ModeChoiceScenario>> allScenarios, String filename) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
            // Write header
            StringBuilder header = new StringBuilder("trip_id");
            for (int i = 1; i <= allScenarios.get(0).size(); i++) {
                header.append(",Scenario ").append(i);
            }
            writer.write(header.toString() + "\n");

            // Write data rows
            for (List<ModeChoiceScenario> scenarios : allScenarios) {
                StringBuilder row = new StringBuilder(scenarios.get(0).tripId); // Assumes all scenarios have the same tripId
                for (ModeChoiceScenario scenario : scenarios) {
                    row.append(",").append(scenario.modeChoice);
                }
                writer.write(row.toString() + "\n");
            }
        }
    }

    private static class Trip {
        String tripId;
        double carUtility, ptUtility, uamUtility;

        public Trip(String tripId, double carUtility, double ptUtility, double uamUtility) {
            this.tripId = tripId;
            this.carUtility = carUtility;
            this.ptUtility = ptUtility;
            this.uamUtility = uamUtility;
        }
    }

    private static class ModeChoiceScenario {
        String tripId;
        String modeChoice;

        public ModeChoiceScenario(String tripId, String modeChoice) {
            this.tripId = tripId;
            this.modeChoice = modeChoice;
        }
    }
}