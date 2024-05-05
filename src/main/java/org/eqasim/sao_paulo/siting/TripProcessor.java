package org.eqasim.sao_paulo.siting;

import java.io.*;
import java.nio.file.*;
import java.util.*;

public class TripProcessor {
    private static final double POOLING_TIME_WINDOW = 2*60; // 5 minutes converted to seconds
    private static final double SEARCH_RADIUS = 1000; // Radius in meters for nearby station search

    public static void main(String[] args) throws IOException {
        // Define the paths to your data files
        Path selectedTripsPath = Paths.get("src/main/java/org/eqasim/sao_paulo/siting/selected_trips.csv");
        Path allTripsPath = Paths.get("scenarios/1-percent/UpdatedFinalTrips.csv");
        Path uamTripsPath = Paths.get("scenarios/1-percent/UAMTravelTimes.csv");
        Path stationsPath = Paths.get("src/main/java/org/eqasim/sao_paulo/siting/utils/Vertiports.csv");

        // Read and process the data
        Map<String, Double> selectedTrips = readSelectedTrips(selectedTripsPath);
        Map<String, String[]> filteredTrips = filterTrips(allTripsPath, selectedTrips);
        Map<Integer, Station> stations = readStations(stationsPath);
        List<UAMTrip> uamTrips = readUAMTrips(uamTripsPath, filteredTrips, stations);

        // Calculate and print pooling statistics
        calculatePoolingStatistics(uamTrips, POOLING_TIME_WINDOW, stations);
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

    private static Map<Integer, Station> readStations(Path filePath) throws IOException {
        Map<Integer, Station> stations = new HashMap<>();
        try (BufferedReader reader = Files.newBufferedReader(filePath)) {
            String line = reader.readLine(); // Skip header
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                Integer stationId = Integer.parseInt(parts[0]);
                double x = Double.parseDouble(parts[2]);
                double y = Double.parseDouble(parts[3]);
                stations.put(stationId, new Station(stationId, x, y));
            }
        }
        return stations;
    }

    private static List<UAMTrip> readUAMTrips(Path filePath, Map<String, String[]> filteredTrips, Map<Integer, Station> stations) throws IOException {
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
                    // Or skip trips where station IDs are not in the stations map
                    if (!stations.containsKey(origStation) || !stations.containsKey(destStation)){
                        System.err.println("We have trips using the vertiports not defined in the vertiport.csv!");
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

    private static void calculatePoolingStatistics(List<UAMTrip> trips, double timeWindow, Map<Integer, Station> stations) {
        Map<String, List<UAMTrip>> groupedTrips = new HashMap<>();
        for (UAMTrip trip : trips) {
            for (UAMTrip otherTrip : trips) {
                if (trip != otherTrip && areStationsNearby(trip.origStation, otherTrip.origStation, stations) &&
                        areStationsNearby(trip.destStation, otherTrip.destStation, stations) &&
                        Math.abs(trip.departureTime - otherTrip.departureTime) <= timeWindow) {

                    String key = trip.origStation + "_" + trip.destStation;
                    groupedTrips.computeIfAbsent(key, k -> new ArrayList<>()).add(trip);
                    break; // Ensure that each trip is only added once per eligible grouping
                }
            }
        }

        int pooledTrips = 0;
        double savedDistance = 0.0;
        double totalIncreasedWaitTime = 0.0;

        for (List<UAMTrip> group : groupedTrips.values()) {
            if (group.size() > 1) {
                group.sort(Comparator.comparingDouble(t -> t.departureTime));
                UAMTrip baseTrip = group.get(0);
                for (int i = 1; i < group.size(); i++) {
                    UAMTrip pooledTrip = group.get(i);
                    savedDistance += baseTrip.flightDistance;
                    pooledTrips++;
                    totalIncreasedWaitTime += (pooledTrip.departureTime - baseTrip.departureTime);
                }
            }
        }

        double averageIncreasedWaitTime = pooledTrips > 0 ? totalIncreasedWaitTime / pooledTrips : 0.0;

        System.out.println("Number of pooled trips: " + pooledTrips);
        System.out.println("Saved flight distance: " + savedDistance);
        System.out.println("Average increased wait time (seconds): " + averageIncreasedWaitTime);
    }

    private static boolean areStationsNearby(int stationId1, int stationId2, Map<Integer, Station> stations) {
        Station s1 = stations.get(stationId1);
        Station s2 = stations.get(stationId2);
        // TODO: Use MATSim to calculate the distance
        double distance = Math.sqrt(Math.pow(s1.x - s2.x, 2) + Math.pow(s1.y - s2.y, 2));
        return distance <= SEARCH_RADIUS;
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

    static class Station {
        int id;
        double x, y;

        Station(int id, double x, double y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }
}
