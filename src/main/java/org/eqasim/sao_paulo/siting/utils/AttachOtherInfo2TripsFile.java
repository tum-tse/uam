package org.eqasim.sao_paulo.siting.utils;

import java.io.*;
import java.util.HashMap;
import java.util.Map;

import org.eqasim.sao_paulo.mode_choice.parameters.SaoPauloModeParameters;
import org.eqasim.sao_paulo.mode_choice.parameters.SaoPauloCostParameters;

import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.TransportMode;
import org.matsim.api.core.v01.population.*;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.population.io.PopulationReader;
import org.matsim.core.scenario.ScenarioUtils;

import org.matsim.households.Household;
import org.matsim.households.Households;
import org.matsim.households.HouseholdsReaderV10;

public class AttachOtherInfo2TripsFile {
    final static boolean updateFinalTrips = true;
    final static double waitTimeForUAMVehilce = 300;

    public static void main(String[] args) {

        String tripPurposeFilePath = "scenarios/1-percent/sao_paulo_population2trips.csv";
        String carFilePath = "scenarios/1-percent/CarTravelTimes.csv";
        String ptFilePath = "scenarios/1-percent/PTTravelTimes.csv";
        String uamFilePath = null;
        if (updateFinalTrips) {
            uamFilePath = "scenarios/1-percent/UAMTravelTimes.csv";
        }
        String outputPath = "scenarios/1-percent/FinalTrips.csv";
        String updatedOutputPath = "scenarios/1-percent/UpdatedFinalTrips.csv";

        String householdFilePath = "/home/tumtse/Documents/haowu/uam/uam/scenarios/1-percent/sao_paulo_households.xml.gz";
        String inputPopulationFile = "scenarios/1-percent/sao_paulo_population.xml.gz"; // specify the path to your population file

        // Setup MATSim configuration and scenario
        Config config = ConfigUtils.createConfig();
        Scenario scenario = ScenarioUtils.createScenario(config);
        PopulationReader reader = new PopulationReader(scenario);
        reader.readFile(inputPopulationFile);
        Population population = scenario.getPopulation();
        Households households = scenario.getHouseholds();
        Map<String, Integer> householdNumberMap = getHouseholdData(households, householdFilePath);


        try (BufferedReader tripPurposeReader = new BufferedReader(new FileReader(tripPurposeFilePath));
             BufferedReader carReader = new BufferedReader(new FileReader(carFilePath));
             BufferedReader ptReader = new BufferedReader(new FileReader(ptFilePath));
             BufferedReader uamReader = new BufferedReader(new FileReader(uamFilePath));
             BufferedWriter writer = new BufferedWriter(new FileWriter(outputPath));
             BufferedWriter updatedWriter = new BufferedWriter(new FileWriter(updatedOutputPath))) {

            // Skip the header line for files
            tripPurposeReader.readLine();
            carReader.readLine();
            ptReader.readLine();
            uamReader.readLine();
                   writer.write("trip_id,originX,originY,destinationX,destinationY,start_time,travel_time_pt,distance_pt,in_vehicle_time_pt,waiting_time_pt,travel_time_car,distance_car,purpose,car_cost,pt_cost,car_utility,pt_utility,uam_utility_fix,car_generalized_cost,pt_generalized_cost,income\n");
            updatedWriter.write("trip_id,originX,originY,destinationX,destinationY,start_time,travel_time_pt,distance_pt,in_vehicle_time_pt,waiting_time_pt,travel_time_car,distance_car,purpose,car_cost,pt_cost,car_utility,pt_utility,uam_utility_fix,car_generalized_cost,pt_generalized_cost,income,uam_utility\n");

            Map<String, String> tripPurposes = new HashMap<>();
            String tripPurposeLine;
            while ((tripPurposeLine = tripPurposeReader.readLine()) != null) {
                String[] tripPurposeData = tripPurposeLine.split(",");
                // Assuming trip_id is at index 0 and trip_purpose is at the last index
                tripPurposes.put(tripPurposeData[0], tripPurposeData[tripPurposeData.length - 1]);
            }

            String carLine;
            String ptLine;
            String uamLine;

            while ((carLine = carReader.readLine()) != null && (ptLine = ptReader.readLine()) != null && (uamLine = uamReader.readLine()) != null) {
                String[] carData = carLine.split(",");
                String[] ptData = ptLine.split(",");
                String[] uamData = uamLine.split(",");

                // Assuming that the trip_id matches in the same line order for both files
                String tripId = carData[0];
                String originX = carData[1];
                String originY = carData[2];
                String destinationX = carData[3];
                String destinationY = carData[4];
                String startTime = carData[5];

                //for pt "null" trips, set pt wait time and in vehicle time to 500000 and pt travel time to 1000000, so that pt's utility is very tiny!
                String description = ptData[10];
                double travelTimePt;
                double inVehicleTimePt;
                double waitingTimePt;
                double distancePt;
                if (description != null){
                    travelTimePt = Double.parseDouble(ptData[8]);
                    inVehicleTimePt = Double.parseDouble(ptData[7]);
                    waitingTimePt = Double.parseDouble(ptData[6]);
                    distancePt = Double.parseDouble(ptData[9]);
                } else {
                    travelTimePt = 1000000;
                    inVehicleTimePt = 500000;
                    waitingTimePt = 500000;
                    distancePt = 0;
                }

                double travelTimeCar = Double.parseDouble(carData[6]);
                double distanceCar = Double.parseDouble(carData[7]);

                String purpose = tripPurposes.get(tripId); // Get purpose from map

                // Get the default parameters
                SaoPauloModeParameters saoPauloModeParametersarameters = SaoPauloModeParameters.buildDefault();
                SaoPauloCostParameters saoPauloCostParametersarameters = SaoPauloCostParameters.buildDefault();
                // Access the car cost per kilometer
                double car_cost = saoPauloCostParametersarameters.carCost_BRL_km*distanceCar;

                // Split the combined ID into an array based on "@"
                String[] parts = tripId.split("@");
                // Extract person ID and trip ID from the parts
                String personId = parts[0];
                String personTripId = parts[1];

                Person person = population.getPersons().get(Id.createPersonId(personId));
                String householdId = person.getAttributes().getAttribute("householdId").toString();
                double householdIncome = (double) person.getAttributes().getAttribute("householdIncome");
                int householdSize = householdNumberMap.get(householdId);
                double income = Double.parseDouble(String.valueOf(householdIncome/householdSize));
                boolean hasPtSubscription = (Boolean) person.getAttributes().getAttribute("hasPtSubscription");

                int ptTransfers = Integer.parseInt(ptData[11]);
                double pt_cost = calculatePtCost(saoPauloCostParametersarameters, ptTransfers, hasPtSubscription); // Calculate PT cost based on transfers and hasSubscription
                double car_utility = calculateCarUtility(saoPauloModeParametersarameters, saoPauloCostParametersarameters, travelTimeCar, distanceCar);
                double pt_utility = calculatePtUtility(saoPauloModeParametersarameters, saoPauloCostParametersarameters, ptTransfers, waitingTimePt, inVehicleTimePt, hasPtSubscription);
                double uam_utility_fix = saoPauloModeParametersarameters.spTaxi.alpha_u; //TODO: maybe need to rethink!
                double vot = income/householdSize/2086; // 2086 = 365*8*(5/7) TODO: check this
                double car_generalized_cost = car_cost + (vot * travelTimeCar);
                double pt_generalized_cost = pt_cost + (vot * travelTimePt);

                String commonData = String.format("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
                        tripId, personId, originX, originY, destinationX, destinationY, startTime, travelTimePt, distancePt,
                        inVehicleTimePt, waitingTimePt, travelTimeCar, distanceCar, purpose, car_cost, pt_cost, car_utility, pt_utility, uam_utility_fix, car_generalized_cost, pt_generalized_cost, income);

                if(updateFinalTrips){
                    //for uam "null" trips, set uam attributes to large values, so that uam's utility is very tiny!
                    double uamFlightDistance = Double.parseDouble(uamData[17]);
                    double uamTravelTime = 0;
                    double proccessTime = 0;
                    String accessMode = "nonUAM";
                    String egressMode = "nonUAM";
                    double access_egress_time = 0;
                    double accessCarDistance;
                    double egressCarDistance;
                    if (description != null){
                        uamTravelTime = Double.parseDouble(uamData[6]);
                        proccessTime = Double.parseDouble(uamData[14]);
                        accessMode = uamData[10];
                        egressMode = uamData[11];
                        access_egress_time = Double.parseDouble(uamData[7]) + Double.parseDouble(uamData[9]);
                        accessCarDistance = Double.parseDouble(uamData[15]);
                        egressCarDistance = Double.parseDouble(uamData[16]);
                    } else {
                        uamTravelTime = 1000000;
                        proccessTime = 500000;
                        access_egress_time = 500000;
                        accessCarDistance = 0;
                        egressCarDistance = 0;
                    }

                    double uam_utility = calculateUamUtility(saoPauloModeParametersarameters, saoPauloCostParametersarameters, hasPtSubscription, uamTravelTime, proccessTime, uamFlightDistance, accessMode, egressMode, access_egress_time, accessCarDistance, egressCarDistance);
                    updatedWriter.write(commonData.substring(0, commonData.length() - 1) + "," + uam_utility + "\n");
                } else {
                    writer.write(commonData);
                }
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static double calculateUamUtility(SaoPauloModeParameters saoPauloModeParametersarameters, SaoPauloCostParameters saoPauloCostParameters, boolean hasPtSubscription, double travelTime, double processTime,  double uamFlightDistance, String accessMode, String egressMode, double access_egress_time, double accessCarDistance, double egressCarDistance) {
        // TODO: UAM utility calculation logic
        double uamUtility;
        double uamUtility_Time;
        double uamUtility_Cost_UAM_Only;
        double uamUtility_Cost_Access_Egress;

        double uamAlpha_u = saoPauloModeParametersarameters.spTaxi.alpha_u;
        double beta_TravelTime_u_min = saoPauloModeParametersarameters.spTaxi.beta_TravelTime_u_min;
        double betaWaitingTime_u_min = saoPauloModeParametersarameters.spTaxi.betaWaitingTime_u_min;
        double betaAccessEgressWalkTime_min = saoPauloModeParametersarameters.spTaxi.betaAccessEgressWalkTime_min;
        double alpha_taxi_city = saoPauloModeParametersarameters.spTaxi.alpha_taxi_city; //Do not distinguish the city trips and rural trips

        uamUtility_Time = beta_TravelTime_u_min*travelTime + betaWaitingTime_u_min*(processTime+waitTimeForUAMVehilce) + betaAccessEgressWalkTime_min*access_egress_time; //TODO: check if the absolute value of betaAccessEgressWalkTime_min is too big.
        uamUtility_Cost_UAM_Only = saoPauloModeParametersarameters.betaCost_u_MU * calculatePureUAMCost(saoPauloCostParameters.taxMinCost_BRL, saoPauloCostParameters.taxiPickUpFee_BRL, saoPauloCostParameters.taxiCostPerkm_BRL, uamFlightDistance);

        // The following block made the assumption that people use pt will only pay the lowest fee since UAM service is newly introduced policy by government.
        int accessPtTransfer;
        int egressPtTransfer;
        if (accessMode.equals(TransportMode.pt)) {
            accessPtTransfer = 0;
        } else {
            accessPtTransfer = -1;
        }
        if (egressMode.equals(TransportMode.pt)) {
            egressPtTransfer = 0;
        } else {
            egressPtTransfer = -1;
        }

        uamUtility_Cost_Access_Egress = saoPauloModeParametersarameters.betaCost_u_MU * ( calculateCarCost(saoPauloModeParametersarameters, saoPauloCostParameters, accessCarDistance) + calculateCarCost(saoPauloModeParametersarameters, saoPauloCostParameters, egressCarDistance) + calculatePtCost(saoPauloCostParameters, accessPtTransfer, hasPtSubscription) + calculatePtCost(saoPauloCostParameters, egressPtTransfer, hasPtSubscription) );
        uamUtility = uamAlpha_u + uamUtility_Time + uamUtility_Cost_UAM_Only + uamUtility_Cost_Access_Egress;
        return uamUtility;
    }
    /**
     * Calculates the cost of Urban Air Mobility (UAM) based on provided cost parameters and distance.
     *
     * @param taxMinCost_BRL The minimum cost in BRL.
     * @param taxiPickUpFee_BRL The pick-up fee for the taxi in BRL.
     * @param taxiCostPerkm_BRL The cost per kilometer in BRL.
     * @param distanceInKm The distance traveled in kilometers.
     * @return The total cost of the UAM in BRL.
     */
    public static double calculatePureUAMCost(double taxMinCost_BRL, double taxiPickUpFee_BRL, double taxiCostPerkm_BRL, double distanceInKm) {
        double totalCost = taxiPickUpFee_BRL + (taxiCostPerkm_BRL * distanceInKm);
        // Ensure the total cost is not below the minimum cost.
        if (totalCost < taxMinCost_BRL) {
            totalCost = taxMinCost_BRL;
        }
        return totalCost;
    }

    private static double calculateCarUtility(SaoPauloModeParameters saoPauloModeParametersarameters, SaoPauloCostParameters saoPauloCostParametersarameters, double travelTime, double travelDistance) {
        double carUtility;
        double carUtility_Time;
        double carUtility_Cost;

        double carAlpha_u = saoPauloModeParametersarameters.car.alpha_u;
        double betaTravelTime_u_min = saoPauloModeParametersarameters.car.betaTravelTime_u_min;
        double constantAccessEgressWalkTime_min = saoPauloModeParametersarameters.car.constantAccessEgressWalkTime_min; //= 0.0;
        double constantParkingSearchPenalty_min = saoPauloModeParametersarameters.car.constantParkingSearchPenalty_min; //= 0.0;
        double alpha_car_city = saoPauloModeParametersarameters.spCar.alpha_car_city; // TODO: We do not distinguish city trip and rural trip

        carUtility_Time = betaTravelTime_u_min*travelTime; //TODO: do not consider constantAccessEgressWalkTime_min and constantParkingSearchPenalty_min
        carUtility_Cost = saoPauloModeParametersarameters.betaCost_u_MU * calculateCarCost(saoPauloModeParametersarameters, saoPauloCostParametersarameters, travelDistance); //TODO: do not consider: EstimatorUtils.interaction(variables.euclideanDistance_km, parameters.referenceEuclideanDistance_km, parameters.lambdaCostEuclideanDistance)   ->  Math.pow(Math.max(minimumValue, value) / reference, exponent)
        carUtility = carAlpha_u + carUtility_Time + carUtility_Cost;
        return carUtility;
    }
    private static double calculateCarCost(SaoPauloModeParameters saoPauloModeParametersarameters, SaoPauloCostParameters saoPauloCostParametersarameters, double travelDistance) {
        return saoPauloModeParametersarameters.betaCost_u_MU * saoPauloCostParametersarameters.carCost_BRL_km * travelDistance;
    }

    private static double calculatePtUtility(SaoPauloModeParameters saoPauloModeParametersarameters, SaoPauloCostParameters saoPauloCostParametersarameters, int transfers, double waitingTime, double inVehicleTime, boolean hasPtSubscription) {
        double ptUtility;
        double ptUtility_Time;
        double ptUtility_Cost;

        // PT
        double alpha_u = saoPauloModeParametersarameters.pt.alpha_u;
        double betaLineSwitch_u = saoPauloModeParametersarameters.pt.betaLineSwitch_u;
        double betaInVehicleTime_u_min = saoPauloModeParametersarameters.pt.betaInVehicleTime_u_min;
        double betaWaitingTime_u_min = saoPauloModeParametersarameters.pt.betaWaitingTime_u_min;
        double betaAccessEgressTime_u_min = saoPauloModeParametersarameters.pt.betaAccessEgressTime_u_min; //TODO: Do not distinguish access, egress time and travel time
        double alpha_pt_city = saoPauloModeParametersarameters.spPT.alpha_pt_city; //= 0.0; //TODO: Do not distinguish city trips and rural trips
        double alpha_age = saoPauloModeParametersarameters.spPT.alpha_age; //= 0.0; //TODO: do not consider age impact

        ptUtility_Time = betaLineSwitch_u*transfers + betaInVehicleTime_u_min*inVehicleTime + betaAccessEgressTime_u_min*waitingTime;
        ptUtility_Cost = saoPauloModeParametersarameters.betaCost_u_MU * calculatePtCost(saoPauloCostParametersarameters, transfers, hasPtSubscription); //TODO: do not consider: EstimatorUtils.interaction(variables.euclideanDistance_km, parameters.referenceEuclideanDistance_km, parameters.lambdaCostEuclideanDistance)   ->  Math.pow(Math.max(minimumValue, value) / reference, exponent)
        ptUtility = alpha_u + ptUtility_Time + ptUtility_Cost;
        return ptUtility;
    }

    private static double calculatePtCost(SaoPauloCostParameters parameters, int transfers, boolean hasPtSubscription) {
        double ptCostPerTrip_0Transfers = parameters.ptCostPerTrip_0Transfers_BRL;
        double ptCostPerTrip_3Transfers = parameters.ptCostPerTrip_3Transfers_BRL;

        if (hasPtSubscription) {
            return 0;
        }

        if (transfers == -1){
            return 500000;
        }

        // If transfers are 3 or more, use the cost for 3 transfers
        if (transfers >= 3) {
            return ptCostPerTrip_3Transfers;
        }

        // Interpolate costs for 0 to 2 transfers
        // Using linear interpolation formula:
        // y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        // where (x1, y1) = (0, ptCostPerTrip_0Transfers) and (x2, y2) = (3, ptCostPerTrip_3Transfers)

        return ptCostPerTrip_0Transfers + transfers * ((ptCostPerTrip_3Transfers - ptCostPerTrip_0Transfers) / 3.0);
    }

    private static Map<String, Integer> getHouseholdData (Households households, String householdFilePath){
        // Step 1: Read Households Data
        HouseholdsReaderV10 reader = new HouseholdsReaderV10(households);
        reader.readFile(householdFilePath);

        // Step 2: Create a Map to store household number
        Map<String, Integer> householdNumberMap = new HashMap<>();

        // Step 3: Process the households data
        for (Household household : households.getHouseholds().values()) {
            householdNumberMap.put(household.getId().toString(), household.getMemberIds().size());
            //System.out.println("Household ID: " + household.getId() + ", Number of Members: " + household.getMemberIds().size());
        }

        // Optionally, print the map to see all data
        //System.out.println("Household Numbers Map: " + householdNumberMap);
        return householdNumberMap;
    }

}