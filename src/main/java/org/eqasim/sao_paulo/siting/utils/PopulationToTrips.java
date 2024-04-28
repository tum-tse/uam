package org.eqasim.sao_paulo.siting.utils;

import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.population.*;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.population.io.PopulationReader;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.utils.io.IOUtils;

import java.io.BufferedWriter;
import java.io.IOException;

public class PopulationToTrips {
    public static void main(String[] args) {
        String inputPopulationFile = "scenarios/1-percent/sao_paulo_population.xml.gz"; // specify the path to your population file
        String outputCsvFile = "scenarios/1-percent/sao_paulo_population2trips.csv"; // specify the output path for the trips CSV file

        // Setup MATSim configuration and scenario
        Config config = ConfigUtils.createConfig();
        Scenario scenario = ScenarioUtils.createScenario(config);
        PopulationReader reader = new PopulationReader(scenario);
        reader.readFile(inputPopulationFile);

        try (BufferedWriter writer = IOUtils.getBufferedWriter(outputCsvFile)) {
            // Write CSV header
            writer.write("trip_id,originX,originY,destinationX,destinationY,start_time,trip_purpose\n"); //,travel_time_pt,distance_pt,in_vehicle_time_pt,waiting_time_pt,travel_time_car,distance_car,purpose,car_cost,pt_cost,car_utility,pt_utility,uam_utility_fix,car_generalized_cost,pt_generalized_cost,income

            for (Person person : scenario.getPopulation().getPersons().values()) {
                Plan plan = person.getSelectedPlan();
                if (plan != null) {
                    // Assuming the plan has at least two activities (like home-work-home)
                    int trip_id = 0;
                    for (int i = 0; i < plan.getPlanElements().size() - 1; i++) {
                        PlanElement pe = plan.getPlanElements().get(i);
                        if (pe instanceof Activity) {
                            trip_id++;
                            Activity origin = (Activity) pe;
                            Leg leg = (Leg) plan.getPlanElements().get(i + 1);
                            Activity destination = (Activity) plan.getPlanElements().get(i + 2);

                            // Write data to CSV
                            writer.write(person.getId().toString()+"@"+trip_id + "," +
                                    origin.getCoord().getX() + "," +
                                    origin.getCoord().getY() + "," +
                                    destination.getCoord().getX() + "," +
                                    destination.getCoord().getY() + "," +
                                    leg.getDepartureTime().seconds() + "," +
                                    destination.getType() + "\n");
                        }
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}