package org.eqasim.sao_paulo.siting;

import java.io.*;
import java.nio.file.*;
import java.util.*;

import java.util.Map;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import net.bhl.matsim.uam.infrastructure.UAMStation;
import net.bhl.matsim.uam.infrastructure.readers.UAMXMLReader;
import net.bhl.matsim.uam.infrastructure.UAMVehicle;
import org.matsim.contrib.dvrp.fleet.DvrpVehicle;
import org.matsim.contrib.dvrp.fleet.FleetSpecificationImpl;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.network.io.MatsimNetworkReader;

public class Utils {
    private static final double SEARCH_RADIUS = 400; // Radius in meters for nearby station search
    private static final int UAM_CAPACITY = 4; // Maximum number of seats in a UAM vehicle
    private static final double TELEPORTATION_SPEED = 20/3.6; // Walking speed in meters per second

    private static final String NULL_VERTIPORT = "NULL_Vertiport";

    public static void main(String[] args) throws IOException {
        DataLoader dataLoader = new DataLoader();
        dataLoader.loadAllData();

        // minutes converted to seconds
        double POOLING_TIME_WINDOW = 10 * 60;
        // Define the paths to your data files
        String pooledTripsPath = "src/main/java/org/eqasim/sao_paulo/siting/output_pooled_trips.csv";

        List<List<UAMTrip>> pooledGroups = poolTrips(dataLoader.uamTrips, POOLING_TIME_WINDOW);
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

    private static List<UAMTrip> readUAMTrips(Path filePath, Map<String, String[]> filteredTrips, Map<Id<UAMStation>, UAMStation> stations) throws IOException {
        List<UAMTrip> trips = new ArrayList<>();
        try (BufferedReader reader = Files.newBufferedReader(filePath)) {
            String line = reader.readLine(); // Skip header
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (filteredTrips.containsKey(parts[0])) {
                    String[] tripDetails = filteredTrips.get(parts[0]);
                    String origStationStringId = parseStationId(parts[12]);
                    String destStationStringId = parseStationId(parts[13]);
                    if (origStationStringId.equals(NULL_VERTIPORT) | origStationStringId.equals(NULL_VERTIPORT)) {
                        continue; // Skip trips where station IDs are NULL_VERTIPORT
                    }

                    Id<UAMStation> origStationId = Id.create(Objects.requireNonNull(origStationStringId), UAMStation.class);
                    Id<UAMStation> destStationId = Id.create(Objects.requireNonNull(destStationStringId), UAMStation.class);
                    // Or skip trips where station IDs are not in the stations map
                    if (!stations.containsKey(origStationId) || !stations.containsKey(destStationId)){
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
                            stations.get(origStationId),
                            stations.get(destStationId),
                            tripDetails[12], // purpose
                            tripDetails[20]  // income
                    );
                    trips.add(trip);
                }
            }
        }
        return trips;
    }
    private static String parseStationId(String stationId) {
        try {
            return (stationId != null) && (!stationId.isEmpty()) && (!stationId.equalsIgnoreCase("null")) ? stationId : NULL_VERTIPORT;
        } catch (NumberFormatException e) {
            return null; // Return null if conversion fails
        }
    }

    private static List<List<UAMTrip>> poolTrips(List<UAMTrip> trips, double timeWindow) {
        Map<String, Set<UAMTrip>> potentialGroups = new HashMap<>();
        for (UAMTrip trip : trips) { // pooled trip candidate
            for (UAMTrip otherTrip : trips) { // base trip
                double accessTimeToStationForBaseTrip = otherTrip.calculateTeleportationTime(otherTrip.origStation);
                double accessTimeToOldStationForPoolTrip = trip.calculateTeleportationTime(trip.origStation);
                double accessTimeToNewStationForPoolTrip = trip.calculateTeleportationTime(otherTrip.origStation);
                double arrivalTimeDifference = (otherTrip.departureTime+accessTimeToStationForBaseTrip - (trip.departureTime+accessTimeToNewStationForPoolTrip));
                if (trip != otherTrip && areStationsNearby(trip.originX, trip.getOriginY(), otherTrip.origStation) &&
                    areStationsNearby(trip.destX, trip.destY, otherTrip.destStation) &&
                    arrivalTimeDifference <= timeWindow && arrivalTimeDifference >= 0) {
                    String key = otherTrip.origStation + "_" + otherTrip.destStation;
                    potentialGroups.computeIfAbsent(key, k -> new HashSet<>()).add(otherTrip);
                    potentialGroups.computeIfAbsent(key, k -> new HashSet<>()).add(trip);

                    // Ensure that each trip is only added once per eligible grouping
                    break; // TODO: need to pool the trips optimally instead of this!
                }
            }
        }

        // Convert Set to List
        List<List<UAMTrip>> listOfListsOfTrips = potentialGroups.values()
                .stream()
                .map(ArrayList::new) // Converts each Set<UAMTrip> to a List<UAMTrip>
                .collect(Collectors.toList());

        // Limit each group to the capacity of the UAM vehicle
        List<List<UAMTrip>> finalGroups = new ArrayList<>();
        for (List<UAMTrip> group : listOfListsOfTrips) {
            if (group.size() > UAM_CAPACITY) {
                group.sort(Comparator.comparingDouble(t -> t.walkingTimeToPooledStation)); // Prioritize by earliest walking time
                finalGroups.add(new ArrayList<>(group.subList(0, UAM_CAPACITY)));
            } else {
                finalGroups.add(group);
            }
        }
        return finalGroups;
    }
    private static boolean areStationsNearby(UAMStation stationId1, UAMStation stationId2) {
        // TODO: Use MATSim to calculate the distance
        double distance = Math.sqrt(Math.pow(stationId1.getLocationLink().getCoord().getX() - stationId2.getLocationLink().getCoord().getX(), 2) + Math.pow(stationId1.getLocationLink().getCoord().getY() - stationId2.getLocationLink().getCoord().getY(), 2));
        return distance <= SEARCH_RADIUS;
    }
    private static boolean areStationsNearby(double x, double y, UAMStation stationId) {
        // TODO: Use MATSim to calculate the distance
        double distance = Math.sqrt(Math.pow(x - stationId.getLocationLink().getCoord().getX(), 2) + Math.pow(y - stationId.getLocationLink().getCoord().getY(), 2));
        return distance <= SEARCH_RADIUS;
    }

    private static void printPoolingStatistics(List<List<UAMTrip>> pooledGroups) {
        int pooledTrips = 0;
        double savedDistance = 0.0;
        double totalIncreasedWaitTime = 0.0;

        for (List<UAMTrip> group : pooledGroups) {
            //identify the base trip which has the earliest departure time
            UAMTrip baseTrip = group.stream().max(Comparator.comparing(UAMTrip::getDepartureTime)).orElse(null);

            for (UAMTrip pooledTrip : group) {
                if (!pooledTrip.equals(baseTrip)) {
                    savedDistance += baseTrip.flightDistance;
                    pooledTrips++;
                    double accessTimeToStationForBaseTrip = baseTrip.calculateTeleportationTime(baseTrip.origStation);
                    double accessTimeToOldStationForPoolTrip = pooledTrip.calculateTeleportationTime(pooledTrip.origStation);
                    double accessTimeToNewStationForPoolTrip = pooledTrip.calculateTeleportationTime(baseTrip.origStation);
                    totalIncreasedWaitTime += (baseTrip.getDepartureTime() + accessTimeToStationForBaseTrip) - (pooledTrip.getDepartureTime() + accessTimeToOldStationForPoolTrip);
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
                    pw.println(String.format("%d,%s,%s,%s,%f,%f,%s,%s,%f,%d",
                            groupID,
                            trip.tripId,
                            trip.origStation.getId().toString(),
                            trip.destStation.getId().toString(),
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
        private final String tripId;
        private final double originX, originY, destX, destY, departureTime, flightDistance;
        private final UAMStation origStation, destStation; // Changed to Integer to handle null values
        private final String purpose, income;
        private double walkingTimeToPooledStation; // Time to walk to the station

        UAMTrip(String tripId, double originX, double originY, double destX, double destY, double departureTime, double flightDistance, UAMStation origStation, UAMStation destStation, String purpose, String income) {
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

        // TODO: Use MATSim to calculate the routes and travel distances
        public double calculateTeleportationDistance(UAMStation station) {
            return Math.sqrt(Math.pow(originX - station.getLocationLink().getCoord().getX(), 2) + Math.pow(originY - station.getLocationLink().getCoord().getY(), 2));
        }
        // TODO: Use MATSim to calculate the routes and travel times
        public double calculateTeleportationTime(UAMStation station) {
            double distance = calculateTeleportationDistance(station);
            walkingTimeToPooledStation = distance / TELEPORTATION_SPEED;
            return walkingTimeToPooledStation;
        }

        // getDepartureTime
        public double getDepartureTime() {
            return departureTime;
        }
        //getOriginX
        public double getOriginX() {
            return originX;
        }
        //getOriginY
        public double getOriginY() {
            return originY;
        }
        //getOriginStation
        public UAMStation getOriginStation() {
            return origStation;
        }
        //getTripId
        public String getTripId() {
            return tripId;
        }
    }

    public static class DataLoader {
        private final Path selectedTripsPath;
        private final Path allTripsPath;
        private final Path uamTripsPath;
        private final String stationsAndVehiclesPath;
        private final String networkPath;

        private Network network;
        private Map<Id<UAMStation>, UAMStation> stations;
        private Map<Id<DvrpVehicle>, UAMVehicle> vehicles;
        private Map<String, Double> selectedTrips;
        private Map<String, String[]> filteredTrips;
        private List<UAMTrip> uamTrips;

        public DataLoader() {
            this.selectedTripsPath = Paths.get("src/main/java/org/eqasim/sao_paulo/siting/selected_trips.csv");
            this.allTripsPath = Paths.get("scenarios/1-percent/UpdatedFinalTrips.csv");
            this.uamTripsPath = Paths.get("scenarios/1-percent/UAMTravelTimes.csv");
            this.stationsAndVehiclesPath = "scenarios/1-percent/uam-scenario/uam_vehicles.xml.gz";
            this.networkPath = "scenarios/1-percent/uam-scenario/uam_network.xml.gz";
        }

        public void loadAllData() throws IOException {
            this.network = NetworkUtils.createNetwork();
            new MatsimNetworkReader(network).readFile(networkPath);

            UAMXMLReader uamReader = new UAMXMLReader(network);
            uamReader.readFile(stationsAndVehiclesPath);
            this.stations = uamReader.getStations();
            this.vehicles = uamReader.getVehicles();

            this.selectedTrips = readSelectedTrips(selectedTripsPath);
            this.filteredTrips = filterTrips(allTripsPath, selectedTrips);
            this.uamTrips = readUAMTrips(uamTripsPath, filteredTrips, stations);
        }

        public List<UAMTrip> getUamTrips() {
            return uamTrips;
        }
        public Map<Id<DvrpVehicle>, UAMVehicle> getVehicles() {
            return vehicles;
        }
        public Map<Id<UAMStation>, UAMStation> getStations() {
            return stations;
        }
    }

}
