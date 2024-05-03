package org.eqasim.sao_paulo.siting.utils;

import java.io.*;
import java.util.*;

import org.eqasim.sao_paulo.siting.initialization.UAMVertiportsInitialization;

public class VertiportExtractor {

    public static void main(String[] args) throws IOException {
        String csvFilePath = "src/main/java/org/eqasim/sao_paulo/siting/initialization/Vertiports.csv";
        String outputFilePath = "src/main/java/org/eqasim/sao_paulo/siting/utils/Vertiports.csv";

        String logLine = "2024-05-02T13:51:17,267  INFO SimulatedAnnealing:84 Best Solution: [69, 135, 103, 145, 26, 170, 168, 20, 7, 169, 14, 183, 71, 90, 189, 164, 48, 41, 98, 153, 28, 24, 80, 39, 190, 9, 138, 172, 106, 11, 63, 199, 16, 32, 38, 147, 137, 6, 40, 99, 159, 30, 155, 64, 72, 150, 37, 109, 187, 65, 55, 177, 29, 53, 96, 101, 134, 161, 195, 144, 52, 129, 175, 158, 12, 18, 58, 107, 174, 84, 108, 191, 97, 42] Best Energy: 1.7196989060294858E8";
        List<Integer> vertiportIds = parseSolution(logLine);
        List<double[]> matchedData = readAndMatchCsv(csvFilePath, vertiportIds);
        try {
            saveClustersAsCSV(matchedData, outputFilePath, UAMVertiportsInitialization.defaultCSVValues);
        } catch (IOException e) {
            System.err.println("There is some problem. Try to use e.printStackTrace();"); // Consider using a logger
        }
    }

    public static List<Integer> parseSolution(String logLine) {
        List<Integer> vertiportIds = new ArrayList<>();
        String prefix = "Best Solution: [";
        int startIndex = logLine.indexOf(prefix) + prefix.length();
        int endIndex = logLine.indexOf("]", startIndex);
        String ids = logLine.substring(startIndex, endIndex);
        String[] splitIds = ids.split(", ");

        for (String id : splitIds) {
            vertiportIds.add(Integer.parseInt(id.trim()));
        }

        return vertiportIds;
    }

    private static List<double[]> readAndMatchCsv(String filePath, List<Integer> vertiportIds) throws IOException {
        List<double[]> matchedData = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String line = reader.readLine(); // Assuming first line is headers and can be skipped.
            while ((line = reader.readLine()) != null) {
                try {
                    String[] values = line.split(",");
                    int stationId = Integer.parseInt(values[0]);
                    if (vertiportIds.contains(stationId)) {
                        double x = Double.parseDouble(values[2]); // x coordinate
                        double y = Double.parseDouble(values[3]); // y coordinate
                        matchedData.add(new double[]{stationId, x, y}); // Include stationId
                    }
                } catch (NumberFormatException | ArrayIndexOutOfBoundsException e) {
                    System.err.println("Skipping malformed line: " + line);
                }
            }
        }
        return matchedData;
    }

    public static void saveClustersAsCSV(List<double[]> centroids, String filePath, String defaultValues) throws IOException {
        FileWriter csvWriter = new FileWriter(filePath);

        // Write header
        csvWriter.append("station_id,station_name,x,y,z,vtol_z,ground_access_capacity,ground_access_freespeed,flight_access_capacity,flight_access_freespeed,preflighttime,postflighttime,defaultwaittime\n");

        // Write centroids data
        for (double[] centroid : centroids) {
            int stationId = (int) centroid[0]; // Extract station ID
            double x = centroid[1];
            double y = centroid[2];
            csvWriter.append(String.format("%d,Station%d,%.3f,%.3f,%s\n", stationId, stationId, x, y, defaultValues));
        }

        csvWriter.flush();
        csvWriter.close();
    }
}