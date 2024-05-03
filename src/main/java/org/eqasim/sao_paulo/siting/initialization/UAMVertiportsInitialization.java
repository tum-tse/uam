package org.eqasim.sao_paulo.siting.initialization;
import smile.clustering.KMeans;

import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.population.Person;
import org.matsim.api.core.v01.population.Activity;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.api.core.v01.population.Population;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.scenario.ScenarioUtils;

import java.util.ArrayList;
import java.util.List;
import java.io.FileWriter;
import java.io.IOException;

public class UAMVertiportsInitialization {

    //z,vtol_z,ground_access_capacity,ground_access_freespeed,flight_access_capacity,flight_access_freespeed,preflighttime,postflighttime,defaultwaittime
    public static final String defaultCSVValues = "0,600,10000,10000,10000,10000,300,300,300";

    public static void main(String[] args) {
        // extract the input for clustering
        String filename = "scenarios/1-percent/sao_paulo_population.xml.gz";
        Config config = ConfigUtils.createConfig();
        config.plans().setInputFile(filename);
        Scenario scenario = ScenarioUtils.loadScenario(config);
        Population population = scenario.getPopulation();

        List<double[]> coordinates = new ArrayList<>();
        int nonHomeFirstActivityCount = 0;
        int totalPersons = population.getPersons().size();

        for (Person person : population.getPersons().values()) {
            PlanElement firstElement = person.getSelectedPlan().getPlanElements().get(0);
            if (firstElement instanceof Activity) {
                Activity activity = (Activity) firstElement;
                if (!"home".equalsIgnoreCase(activity.getType())) {
                    nonHomeFirstActivityCount++;
                } else {
                    double x = activity.getCoord().getX();
                    double y = activity.getCoord().getY();
                    coordinates.add(new double[]{x, y});
                }
            }
        }

        double nonHomeFirstActivityRatio = (double) nonHomeFirstActivityCount / totalPersons;
        if (nonHomeFirstActivityRatio > 0.10) {
            throw new RuntimeException("home is not the 90% people's first location");
        }

        double[][] data = coordinates.toArray(new double[0][]); // convert List into double[][] as Clustering input


        // KMeansPlusPlusSmile
        int k = 200;
        KMeans kmeans = KMeans.fit(data, k/*, KMeans.Initialization.K_MEANS_PLUS_PLUS*/);

/*        System.out.println("Cluster labels:");
        for (int label : kmeans.y) {
            System.out.println(label);
        }*/

        // Save clusters to CSV
        try {
            saveClustersAsCSV(kmeans.centroids, "src/main/java/org/eqasim/sao_paulo/siting/initialization/stations.csv", defaultCSVValues);
            saveClustersAsCSVForOptimization(kmeans.centroids, "src/main/java/org/eqasim/sao_paulo/siting/initialization/Vertiports.csv");
        } catch (IOException e) {
            System.err.println("Error while saving CSV: " + e.getMessage());
        }
    }

    private static void saveClustersAsCSV(double[][] centroids, String filePath, String defaultValues) throws IOException {
        FileWriter csvWriter = new FileWriter(filePath);

        // Write header
        csvWriter.append("station_id,station_name,x,y,z,vtol_z,ground_access_capacity,ground_access_freespeed,flight_access_capacity,flight_access_freespeed,preflighttime,postflighttime,defaultwaittime\n");

        // Write centroids data
        for (int i = 0; i < centroids.length; i++) {
            double x = centroids[i][0];
            double y = centroids[i][1];
            int stationId = i + 1;
            csvWriter.append(String.format("%d,Station%d,%.3f,%.3f,%s\n", stationId, stationId, x, y, defaultValues));
        }

        csvWriter.flush();
        csvWriter.close();
    }
    private static void saveClustersAsCSVForOptimization(double[][] centroids, String filePath) throws IOException {
        FileWriter csvWriter = new FileWriter(filePath);

        // Write header
        csvWriter.append("vertiportID, vertiportX, vertiportY, ConstructionCost\n");

        // Write centroids data
        for (int i = 0; i < centroids.length; i++) {
            double x = centroids[i][0];
            double y = centroids[i][1];
            double cost = 1.0;
            csvWriter.append(String.format("%d,%.3f,%.3f,%.1f\n", i + 1, x, y, cost));
        }

        csvWriter.flush();
        csvWriter.close();
    }
}
