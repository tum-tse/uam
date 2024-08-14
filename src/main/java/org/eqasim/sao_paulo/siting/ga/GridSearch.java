package org.eqasim.sao_paulo.siting.ga;

import weka.classifiers.Evaluation;
import weka.classifiers.functions.SimpleLinearRegression;
import weka.core.DenseInstance;
import weka.core.Instances;
import weka.core.Attribute;

import java.util.ArrayList;

public class GridSearch {

    public static void main(String[] args) throws Exception {

        // Define the attributes
        ArrayList<Attribute> attributes = new ArrayList<>();
        attributes.add(new Attribute("poolingTimeWindow"));
        attributes.add(new Attribute("searchRadiusOrigin"));
        attributes.add(new Attribute("searchRadiusDestination"));
        attributes.add(new Attribute("FitnessScore"));

        // Create the dataset with these attributes
        Instances dataset = new Instances("OptimizationData", attributes, 0);
        dataset.setClassIndex(dataset.numAttributes() - 1);

        // Example loop to optimize parameters
        for (double ptw = 5.0; ptw <= 15.0; ptw += 1.0) {
            for (double sro = 500; sro <= 3000; sro += 500) {
                for (double srd = 500; srd <= 3000; srd += 500) {

                    // Prepare the arguments you want to pass to the MultiObjectiveNSGAII main method
                    String[] multiObjectiveArgs = {
                            String.valueOf(ptw), // BUFFER_END_TIME
                            String.valueOf(sro), // SEARCH_RADIUS_ORIGIN
                            String.valueOf(srd), // SEARCH_RADIUS_DESTINATION
                            String.valueOf(false)  // ENABLE_LOCAL_SEARCH
                    };
                    // Evaluate the fitness score using the parameters
                    double fitnessScore = MultiObjectiveNSGAII.callAlgorithm(multiObjectiveArgs);

                    // Add the instance to the dataset
                    double[] values = {ptw, sro, srd, fitnessScore};
                    dataset.add(new DenseInstance(1.0, values));
                }
            }
        }

        // Use Weka to find the best parameters using a simple linear regression
        SimpleLinearRegression slr = new SimpleLinearRegression();
        slr.buildClassifier(dataset);

        // Evaluate the model
        Evaluation eval = new Evaluation(dataset);
        eval.evaluateModel(slr, dataset);

        // Output the results
        System.out.println("Best fitness score: " + eval.meanAbsoluteError());
    }
}