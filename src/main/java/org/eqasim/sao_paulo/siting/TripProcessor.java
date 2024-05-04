package org.eqasim.sao_paulo.siting;

import java.io.*;
import java.nio.file.*;
import java.util.*;

public class TripProcessor {
    private static final double POOLING_TIME_WINDOW = 5*60;

    public static void main(String[] args) throws IOException {
        // Define the paths to your data files
        Path selectedTripsPath = Paths.get("/home/tumtse/Documents/haowu/uam/uam/src/main/java/org/eqasim/sao_paulo/siting/selected_trips.csv");
        Path allTripsPath = Paths.get("/home/tumtse/Documents/haowu/uam/uam/scenarios/1-percent/UpdatedFinalTrips.csv");
        Path uamTripsPath = Paths.get("/home/tumtse/Documents/haowu/uam/uam/scenarios/1-percent/UAMTravelTimes.csv");

        // Read and process the data
        Map<String, Double> selectedTrips = readSelectedTrips(selectedTripsPath);
        Map<String, String[]> filteredTrips = filterTrips(allTripsPath, selectedTrips);
        List<UAMTrip> uamTrips = readUAMTrips(uamTripsPath, filteredTrips);

        // Calculate and print pooling statistics
        calculatePoolingStatistics(uamTrips, POOLING_TIME_WINDOW); // Pooling within 30-minute window
    }

    private static Map<String, Double> readSelectedTrips(Path filePath) throws IOException {
        Map<String, Double> trips = new HashMap<>();
        try (BufferedReader reader = Files.newBufferedReader(filePath)) {
            String line = reader.readLine(); // Skip header
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                trips.put(parts[0], Double.parseDouble(parts[1]));
            }
        }
        return trips;
    }

    private static Map<String, String[]> filterTrips(Path filePath, Map<String, Double> selectedTrips) throws IOException {
        Map<String, String[]> filteredTrips = new HashMap<>();
        try (BufferedReader reader = Files.newBufferedReader(filePath)) {
            String line = reader.readLine(); // Skip header
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (selectedTrips.containsKey(parts[0])) {
                    filteredTrips.put(parts[0], parts);
                }
            }
        }
        return filteredTrips;
    }

    private static List<UAMTrip> readUAMTrips(Path filePath, Map<String, String[]> filteredTrips) throws IOException {
        List<UAMTrip> trips = new ArrayList<>();
        try (BufferedReader reader = Files.newBufferedReader(filePath)) {
            String line = reader.readLine(); // Skip header
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (filteredTrips.containsKey(parts[0])) {
                    String[] tripDetails = filteredTrips.get(parts[0]);

                    Integer origStation = parseStationId(parts[12]);
                    Integer destStation = parseStationId(parts[13]);

                    if (origStation == null || destStation == null) {
                        continue; // Skip trips where station IDs are null
                    }

                    UAMTrip trip = new UAMTrip(
                            parts[0],
                            Double.parseDouble(parts[1]),
                            Double.parseDouble(parts[2]),
                            Double.parseDouble(parts[3]),
                            Double.parseDouble(parts[4]),
                            Double.parseDouble(parts[5]),
                            Double.parseDouble(parts[17]),
                            origStation,
                            destStation,
                            tripDetails[12], // purpose
                            tripDetails[20]  // income
                    );
                    trips.add(trip);
                }
            }
        }
        return trips;
    }

    private static Integer parseStationId(String stationId) {
        try {
            return stationId != null && !stationId.isEmpty() && !stationId.equalsIgnoreCase("null") ? Integer.valueOf(stationId) : null;
        } catch (NumberFormatException e) {
            return null; // Return null if conversion fails
        }
    }

    private static void calculatePoolingStatistics(List<UAMTrip> trips, double timeWindow) {
        Map<String, List<UAMTrip>> groupedTrips = new HashMap<>();
        for (UAMTrip trip : trips) {
            if (trip.origStation == null || trip.destStation == null) {
                continue; // Skip trips with null stations
            }
            String key = trip.origStation + "_" + trip.destStation;
            groupedTrips.computeIfAbsent(key, k -> new ArrayList<>()).add(trip);
        }

        int pooledTrips = 0;
        double savedDistance = 0.0;
        double totalIncreasedWaitTime = 0.0;

        for (List<UAMTrip> group : groupedTrips.values()) {
            group.sort(Comparator.comparingDouble(t -> t.departureTime));

            for (int i = 0; i < group.size(); i++) {
                UAMTrip baseTrip = group.get(i);
                for (int j = i + 1; j < group.size() && (group.get(j).departureTime - baseTrip.departureTime) <= timeWindow; j++) {
                    UAMTrip possiblePooledTrip = group.get(j);
                    savedDistance += baseTrip.flightDistance;
                    pooledTrips++;
                    totalIncreasedWaitTime += (possiblePooledTrip.departureTime - baseTrip.departureTime);
                }
            }
        }

        double averageIncreasedWaitTime = pooledTrips > 0 ? totalIncreasedWaitTime / pooledTrips : 0.0;

        System.out.println("Number of pooled trips: " + pooledTrips);
        System.out.println("Saved flight distance: " + savedDistance);
        System.out.println("Average increased wait time (seconds): " + averageIncreasedWaitTime);
    }

    static class UAMTrip {
        String tripId;
        double originX, originY, destX, destY, departureTime, flightDistance;
        Integer origStation, destStation; // Changed to Integer to handle null values
        String purpose, income;

        UAMTrip(String tripId, double originX, double originY, double destX, double destY, double departureTime, double flightDistance, Integer origStation, Integer destStation, String purpose, String income) {
            this.tripId = tripId;
            this.originX = originX;
            this.originY = originY;
            this.destX = destX;
            this.destY = destY;
            this.departureTime = departureTime;
            this.flightDistance = flightDistance;
            this.origStation = origStation;
            this.destStation = destStation;
            this.purpose = purpose;
            this.income = income;
        }
    }
}
