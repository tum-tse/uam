package org.eqasim.sao_paulo.siting;

import java.io.*;
import java.nio.file.*;
import java.util.*;

import java.util.Map;

import net.bhl.matsim.uam.infrastructure.UAMStation;
import net.bhl.matsim.uam.infrastructure.readers.UAMXMLReader;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.network.io.MatsimNetworkReader;

public class Utils {
    private static final double SEARCH_RADIUS = 1000; // Radius in meters for nearby station search
    private static final int UAM_CAPACITY = 4; // Maximum number of seats in a UAM vehicle
    private static final double Teleportation_SPEED = 20/3.6; // Walking speed in meters per second

    public static void main(String[] args) throws IOException {
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();

        // minutes converted to seconds
        double POOLING_TIME_WINDOW = 10 * 60;
        // Define the paths to your data files
        String pooledTripsPath = "src/main/java/org/eqasim/sao_paulo/siting/output_pooled_trips.csv";

        List<List<UAMTrip>> pooledGroups = poolTrips(dataLoader.uamTrips, POOLING_TIME_WINDOW, dataLoader.stations);
        // Calculate and print pooling statistics
        printPoolingStatistics(pooledGroups);
        writePoolingResultsToCSV(pooledGroups, pooledTripsPath);
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

    private static Map<Integer, Station> readStations(String filePath, Network network) throws IOException {
        UAMXMLReader uamReader = new UAMXMLReader(network);
        uamReader.readFile(filePath); // Assuming the file is locally accessible and this function handles it
        Map<Integer, Station> stationMap = new HashMap<>();

        for (Map.Entry<Id<UAMStation>, UAMStation> entry : uamReader.getStations().entrySet()) {
            UAMStation uamStation = entry.getValue();
            Link locationLink = uamStation.getLocationLink();
            Station station = new Station(entry.getKey().toString(), locationLink.getCoord().getX(), locationLink.getCoord().getY());
            stationMap.put(Integer.parseInt(entry.getKey().toString()), station);
        }

        return stationMap;
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
                            stations.get(origStation),
                            stations.get(destStation),
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

    private static List<List<UAMTrip>> poolTrips(List<UAMTrip> trips, double timeWindow, Map<Integer, Station> stations) {
        Map<String, List<UAMTrip>> potentialGroups = new HashMap<>();
        for (UAMTrip trip : trips) {
            for (UAMTrip otherTrip : trips) {
                if (trip != otherTrip && areStationsNearby(trip.origStation, otherTrip.origStation, stations) &&
                        areStationsNearby(trip.destStation, otherTrip.destStation, stations) &&
                        Math.abs(trip.departureTime - otherTrip.departureTime) <= timeWindow) {
                    trip.calculateTeleportationTime(otherTrip.origStation); // Calculate walking time to station

                    String key = trip.origStation + "_" + trip.destStation;
                    potentialGroups.computeIfAbsent(key, k -> new ArrayList<>()).add(trip);
                    // Ensure that each trip is only added once per eligible grouping
                    break; // TODO: need to pool the trips optimally instead of this!
                }
            }
        }

        // Limit each group to the capacity of the UAM vehicle
        List<List<UAMTrip>> finalGroups = new ArrayList<>();
        for (List<UAMTrip> group : potentialGroups.values()) {
            if (group.size() > UAM_CAPACITY) {
                group.sort(Comparator.comparingDouble(t -> t.walkingTimeToPooledStation)); // Prioritize by earliest walking time
                finalGroups.add(new ArrayList<>(group.subList(0, UAM_CAPACITY)));
            } else {
                finalGroups.add(group);
            }
        }
        return finalGroups;
    }
    private static boolean areStationsNearby(Station stationId1, Station stationId2, Map<Integer, Station> stations) {
        // TODO: Use MATSim to calculate the distance
        double distance = Math.sqrt(Math.pow(stationId1.x - stationId2.x, 2) + Math.pow(stationId1.y - stationId2.y, 2));
        return distance <= SEARCH_RADIUS;
    }

    private static void printPoolingStatistics(List<List<UAMTrip>> pooledGroups) {
        int pooledTrips = 0;
        double savedDistance = 0.0;
        double totalIncreasedWaitTime = 0.0;

        for (List<UAMTrip> group : pooledGroups) {
            UAMTrip baseTrip = group.get(0);
            for (UAMTrip pooledTrip : group) {
                if (!pooledTrip.equals(baseTrip)) {
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

    private static void writePoolingResultsToCSV(List<List<UAMTrip>> pooledGroups, String filePath) throws FileNotFoundException {
        try (PrintWriter pw = new PrintWriter(new File(filePath))) {
            pw.println("Group ID,Trip ID,Origin Station ID,Destination Station ID,Departure Time,Arrival Time at Station,Purpose,Income,Walking Time to Station,Total Trips in Group");
            int groupID = 1;
            for (List<UAMTrip> group : pooledGroups) {
                for (UAMTrip trip : group) {
                    pw.println(String.format("%d,%s,%d,%d,%f,%f,%s,%s,%f,%d",
                            groupID,
                            trip.tripId,
                            trip.origStation.id,
                            trip.destStation.id,
                            trip.departureTime,
                            trip.departureTime - trip.walkingTimeToPooledStation, // Arrival time at station
                            trip.purpose,
                            trip.income,
                            trip.walkingTimeToPooledStation,
                            group.size()));
                }
                groupID++;
            }
        }
    }

    public static class UAMTrip {
        String tripId;
        double originX, originY, destX, destY, departureTime, flightDistance;
        Station origStation, destStation; // Changed to Integer to handle null values
        String purpose, income;
        double walkingTimeToPooledStation; // Time to walk to the station

        UAMTrip(String tripId, double originX, double originY, double destX, double destY, double departureTime, double flightDistance, Station origStation, Station destStation, String purpose, String income) {
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

        // TODO: Use MATSim to calculate the routes and travel times
        void calculateTeleportationTime(Station station) {
            double distance = Math.sqrt(Math.pow(originX - station.x, 2) + Math.pow(originY - station.y, 2));
            walkingTimeToPooledStation = distance / Teleportation_SPEED;
        }
    }

    static class Station {
        int id;
        double x, y;

        Station(String id, double x, double y) {
            this.id = Integer.parseInt(id);
            this.x = x;
            this.y = y;
        }
    }

    public static class DataLoader {
        private final Path selectedTripsPath;
        private final Path allTripsPath;
        private final Path uamTripsPath;
        private final String stationsPath;
        private final String networkPath;

        private Network network;
        private Map<Integer, Station> stations;
        private Map<String, Double> selectedTrips;
        private Map<String, String[]> filteredTrips;
        private List<UAMTrip> uamTrips;

        public DataLoader() {
            this.selectedTripsPath = Paths.get("src/main/java/org/eqasim/sao_paulo/siting/selected_trips.csv");
            this.allTripsPath = Paths.get("scenarios/1-percent/UpdatedFinalTrips.csv");
            this.uamTripsPath = Paths.get("scenarios/1-percent/UAMTravelTimes.csv");
            this.stationsPath = "scenarios/1-percent/uam-scenario/uam_vehicles.xml.gz";
            this.networkPath = "scenarios/1-percent/uam-scenario/uam_network.xml.gz";
        }

        public void loadAllData() throws IOException {
            this.network = NetworkUtils.createNetwork();
            new MatsimNetworkReader(network).readFile(networkPath);
            this.stations = Utils.readStations(stationsPath, network);
            this.selectedTrips = Utils.readSelectedTrips(selectedTripsPath);
            this.filteredTrips = Utils.filterTrips(allTripsPath, selectedTrips);
            this.uamTrips = Utils.readUAMTrips(uamTripsPath, filteredTrips, stations);
        }

        public List<UAMTrip> getUamTrips() {
            return uamTrips;
        }
    }

}
