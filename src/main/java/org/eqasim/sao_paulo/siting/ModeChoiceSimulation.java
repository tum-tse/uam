package org.eqasim.sao_paulo.siting;

import java.io.*;
import java.util.*;

public class ModeChoiceSimulation {
    private static final int SCENARIOS_COUNT = 100;
    private static final long SEED = 4711; // MATSim default Random Seed

    public static void main(String[] args) throws IOException {
        List<Trip> trips = readTrips("path_to_your_trip_data.csv");
        Random random = new Random(SEED); // Set the seed here
        List<ModeChoiceScenario> scenarios = generateScenarios(trips, SCENARIOS_COUNT, random);
        saveScenarios(scenarios, "mode_choice_scenarios.csv");
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

    private static List<ModeChoiceScenario> generateScenarios(List<Trip> trips, int numScenarios, Random random) {
        List<ModeChoiceScenario> scenarios = new ArrayList<>();
        for (Trip trip : trips) {
            for (int i = 0; i < numScenarios; i++) {
                String modeChoice = predictModeChoice(trip, random);
                scenarios.add(new ModeChoiceScenario(trip.tripId, modeChoice));
            }
        }
        return scenarios;
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

    private static void saveScenarios(List<ModeChoiceScenario> scenarios, String filename) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
            writer.write("trip_id,mode_choice\n");
            for (ModeChoiceScenario scenario : scenarios) {
                writer.write(scenario.tripId + "," + scenario.modeChoice + "\n");
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