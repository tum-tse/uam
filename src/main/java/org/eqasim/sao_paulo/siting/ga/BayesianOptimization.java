package org.eqasim.sao_paulo.siting.ga;

import weka.classifiers.functions.GaussianProcesses;
import weka.core.DenseInstance;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.Attribute;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.Callable;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class BayesianOptimization {

    private GaussianProcesses gaussianProcess;
    private Instances dataset;
    private final long SEED = 4711; // MATSim default Random Seed
    private final Random rand = new Random(SEED);
    private final int NUM_THREADS = Runtime.getRuntime().availableProcessors();
    private final ExecutorService executorService = Executors.newFixedThreadPool(NUM_THREADS);

    // Initialize with empty dataset
    private final ParameterRange poolingTimeWindowRange;
    private final ParameterRange searchRadiusOriginRange;
    private final ParameterRange searchRadiusDestinationRange;

    public BayesianOptimization(ParameterRange poolingTimeWindowRange,
                                             ParameterRange searchRadiusOriginRange,
                                             ParameterRange searchRadiusDestinationRange) throws Exception {
        this.poolingTimeWindowRange = poolingTimeWindowRange;
        this.searchRadiusOriginRange = searchRadiusOriginRange;
        this.searchRadiusDestinationRange = searchRadiusDestinationRange;

        ArrayList<Attribute> attributes = new ArrayList<>();
        attributes.add(new Attribute("pooling_time_window"));
        attributes.add(new Attribute("search_radius_origin"));
        attributes.add(new Attribute("search_radius_destination"));
        attributes.add(new Attribute("performance_metric"));

        dataset = new Instances("OptimizationDataset", attributes, 0);
        dataset.setClassIndex(3);

        // Initialize GaussianProcesses but don't build it yet
        gaussianProcess = new GaussianProcesses();

        // Add initial data points
        addInitialDataPoints();
    }

    // Add a fixed number of initial data points
    private void addInitialDataPoints() throws Exception {
        int[][] initialPoints = {
                {poolingTimeWindowRange.getMinValue(), searchRadiusOriginRange.getMinValue(), searchRadiusDestinationRange.getMinValue()},
                {poolingTimeWindowRange.getMaxValue(), searchRadiusOriginRange.getMaxValue(), searchRadiusDestinationRange.getMaxValue()},
                {poolingTimeWindowRange.getMidValue(), searchRadiusOriginRange.getMidValue(), searchRadiusDestinationRange.getMidValue()}
        };

        for (int[] point : initialPoints) {
            double performanceMetric = evaluateParameters(point[0], point[1], point[2]);
            addDataPoint(point[0], point[1], point[2], performanceMetric);
        }
    }

    // Add a new data point to the dataset
    public void addDataPoint(int poolingTimeWindow, int searchRadiusOrigin, int searchRadiusDestination, double performanceMetric) throws Exception {
        double[] values = new double[]{poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination, performanceMetric};
        Instance instance = new DenseInstance(1.0, values);
        dataset.add(instance);

        // Build the classifier after adding a new data point
        gaussianProcess.buildClassifier(dataset);
    }

    // Predict performance for a given set of parameters
    public double predictPerformance(int poolingTimeWindow, int searchRadiusOrigin, int searchRadiusDestination) throws Exception {
        if (dataset.numInstances() == 0) {
            throw new IllegalStateException("Cannot predict performance without any training data.");
        }
        double[] values = new double[]{poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination, Double.NaN};
        Instance instance = new DenseInstance(1.0, values);
        instance.setDataset(dataset);
        return gaussianProcess.classifyInstance(instance);
    }

    // Deterministic acquisition function (Upper Confidence Bound)
    private double acquisitionFunction(int poolingTimeWindow, int searchRadiusOrigin, int searchRadiusDestination) throws Exception {
        double mean = predictPerformance(poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination);
        double std = Math.sqrt(gaussianProcess.getStandardDeviation(new DenseInstance(1.0, new double[]{poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination, Double.NaN})));
        return mean + 2 * std;  // UCB with kappa = 2
    }

    // Parallelized optimization method
    public int[] optimizeParameters(int iterations) throws Exception {
        AtomicReference<Double> bestPerformance = new AtomicReference<>(Double.NEGATIVE_INFINITY);
        AtomicReference<int[]> bestParams = new AtomicReference<>(new int[3]);

        for (int i = 0; i < iterations; i++) {
            List<Future<OptimizationResult>> futures = new ArrayList<>();
            for (int ptw = poolingTimeWindowRange.getMinValue(); ptw <= poolingTimeWindowRange.getMaxValue(); ptw++) {
                for (int sro = searchRadiusOriginRange.getMinValue(); sro <= searchRadiusOriginRange.getMaxValue(); sro += 100) {
                    for (int srd = searchRadiusDestinationRange.getMinValue(); srd <= searchRadiusDestinationRange.getMaxValue(); srd += 100) {
                        final int finalPtw = ptw;
                        final int finalSro = sro;
                        final int finalSrd = srd;
                        futures.add(executorService.submit(new Callable<OptimizationResult>() {
                            @Override
                            public OptimizationResult call() throws Exception {
                                double acquisitionValue = acquisitionFunction(finalPtw, finalSro, finalSrd);
                                return new OptimizationResult(finalPtw, finalSro, finalSrd, acquisitionValue);
                            }
                        }));
                    }
                }
            }

            OptimizationResult bestResult = null;
            double bestAcquisitionValue = Double.NEGATIVE_INFINITY;

            for (Future<OptimizationResult> future : futures) {
                OptimizationResult result = future.get();
                if (result.performance > bestAcquisitionValue) {
                    bestResult = result;
                    bestAcquisitionValue = result.performance;
                }
            }

            if (bestResult != null) {
                double actualPerformance = evaluateParameters(bestResult.poolingTimeWindow, bestResult.searchRadiusOrigin, bestResult.searchRadiusDestination);
                addDataPoint(bestResult.poolingTimeWindow, bestResult.searchRadiusOrigin, bestResult.searchRadiusDestination, actualPerformance);

                if (actualPerformance > bestPerformance.get()) {
                    bestPerformance.set(actualPerformance);
                    bestParams.set(new int[]{bestResult.poolingTimeWindow, bestResult.searchRadiusOrigin, bestResult.searchRadiusDestination});
                }
            }
        }

        executorService.shutdown();
        return bestParams.get();
    }

    private double evaluateParameters(int poolingTimeWindow, int searchRadiusOrigin, int searchRadiusDestination) throws IOException, InterruptedException {
        double[] actualPerformance = MultiObjectiveNSGAII.callAlgorithm(new String[]{
                String.valueOf(poolingTimeWindow),
                String.valueOf(searchRadiusOrigin),
                String.valueOf(searchRadiusDestination),
                String.valueOf(false)
        });
        return actualPerformance[3];
    }

    private static class OptimizationResult {
        final int poolingTimeWindow;
        final int searchRadiusOrigin;
        final int searchRadiusDestination;
        final double performance;

        OptimizationResult(int ptw, int sro, int srd, double perf) {
            poolingTimeWindow = ptw;
            searchRadiusOrigin = sro;
            searchRadiusDestination = srd;
            performance = perf;
        }
    }

    public static class ParameterRange {
        private final int min;
        private final int max;

        public ParameterRange(int min, int max) {
            this.min = min;
            this.max = max;
        }

        public int getMinValue() {
            return min;
        }

        public int getMaxValue() {
            return max;
        }

        public int getMidValue() {
            return min + (max - min) / 2;
        }
    }

    public static void main(String[] args) throws Exception {
        ParameterRange poolingTimeWindowRange = new ParameterRange(5, 15);
        ParameterRange searchRadiusOriginRange = new ParameterRange(0, 5000);
        ParameterRange searchRadiusDestinationRange = new ParameterRange(0, 5000);

        BayesianOptimization optimization = new BayesianOptimization(
                poolingTimeWindowRange, searchRadiusOriginRange, searchRadiusDestinationRange);

        int[] bestParams = optimization.optimizeParameters(10);
        System.out.println("Best Parameters: Pooling Time Window = " + bestParams[0] +
                ", Search Radius Origin = " + bestParams[1] +
                ", Search Radius Destination = " + bestParams[2]);
    }
}
