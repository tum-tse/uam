package org.eqasim.sao_paulo.siting.utils;

import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.population.*;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.population.io.PopulationReader;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.utils.io.IOUtils;
import org.matsim.core.router.TripStructureUtils; // Import the TripStructureUtils
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.List;

public class PopulationToTrips {
    public static void main(String[] args) {
        String inputPopulationFile = "scenarios/1-percent/sao_paulo_population.xml.gz"; // specify the path to your population file
        String outputCsvFile = "scenarios/1-percent/sao_paulo_population2trips.csv"; // specify the output path for the trips CSV file

        // Setup MATSim configuration and scenario
        Config config = ConfigUtils.createConfig();
        Scenario scenario = ScenarioUtils.createScenario(config);
        new PopulationReader(scenario).readFile(inputPopulationFile);

        try (BufferedWriter writer = IOUtils.getBufferedWriter(outputCsvFile)) {
            writer.write("trip_id,originX,originY,destinationX,destinationY,start_time,trip_purpose\n");

            for (Person person : scenario.getPopulation().getPersons().values()) {
                Plan plan = person.getSelectedPlan();
                if (plan != null) {
                    int trip_id = 0;
                    List<TripStructureUtils.Trip> trips = TripStructureUtils.getTrips(plan);
                    for (TripStructureUtils.Trip trip : trips) {
                        trip_id++;
                        Activity origin = trip.getOriginActivity();
                        Activity destination = trip.getDestinationActivity();
                        //Leg leg = trip.getLegsOnly().get(0);
                        double departureTime = TripStructureUtils.getDepartureTime(trip).seconds();

                        writer.write(person.getId().toString() + "@" + trip_id + "," +
                                origin.getCoord().getX() + "," +
                                origin.getCoord().getY() + "," +
                                destination.getCoord().getX() + "," +
                                destination.getCoord().getY() + "," +
                                departureTime + "," +
                                destination.getType() + "\n");
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}