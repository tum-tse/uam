package org.eqasim.sao_paulo.scenario;

import net.bhl.matsim.uam.scenario.RunCreateUAMBeelineScenario;

/*
Change pom.xml to version 4.1.3 of MATSim-UAM for running this class (There is no codes changes inbetween (between 3.0.0 and 4.1.3) related to the functions used in this class)
 */
public class CreateUAMScenario {
    public static void main(String[] args) {
        // "ARGS: config.xml* uam-stations.csv* uam-link-freespeed* uam-link-capacity* uam-vehicles.csv"
        String[] scenarioArgs = {"scenarios/1-percent/sao_paulo_config.xml", "src/main/java/org/eqasim/sao_paulo/siting/initialization/stations.csv", "100", "1000000", "src/main/java/org/eqasim/sao_paulo/siting/initialization/vehicles.csv"};
        RunCreateUAMBeelineScenario.main(scenarioArgs);
    }
}
