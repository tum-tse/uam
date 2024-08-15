package org.eqasim.sao_paulo.siting.ga;

import weka.classifiers.functions.GaussianProcesses;
import weka.core.DenseInstance;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.Attribute;

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

        // Add initial data point to avoid empty dataset
        addInitialDataPoint();
    }

    // Add an initial data point
    private void addInitialDataPoint() throws Exception {
        int poolingTimeWindow = poolingTimeWindowRange.getRandomValue(rand);
        int searchRadiusOrigin = searchRadiusOriginRange.getRandomValue(rand);
        int searchRadiusDestination = searchRadiusDestinationRange.getRandomValue(rand);
        double performanceMetric = Math.random() * 100; // Example initial performance

        addDataPoint(poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination, performanceMetric);
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

    // Parallelized optimization method
    public int[] optimizeParameters(int iterations) throws Exception {
        AtomicReference<Double> bestPerformance = new AtomicReference<>(Double.NEGATIVE_INFINITY);
        AtomicReference<int[]> bestParams = new AtomicReference<>(new int[3]);

        for (int i = 0; i < iterations; i += NUM_THREADS) {
            List<Future<OptimizationResult>> futures = new ArrayList<>();
            for (int j = 0; j < NUM_THREADS && i + j < iterations; j++) {
                futures.add(executorService.submit(new Callable<OptimizationResult>() {
                    @Override
                    public OptimizationResult call() throws Exception {
                        int poolingTimeWindow = poolingTimeWindowRange.getRandomValue(rand);
                        int searchRadiusOrigin = searchRadiusOriginRange.getRandomValue(rand);
                        int searchRadiusDestination = searchRadiusDestinationRange.getRandomValue(rand);

                        double predictedPerformance = predictPerformance(poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination);

                        if (predictedPerformance > bestPerformance.get() || rand.nextDouble() < 0.1) {
                            double[] actualPerformance = MultiObjectiveNSGAII.callAlgorithm(new String[]{
                                    String.valueOf(poolingTimeWindow),
                                    String.valueOf(searchRadiusOrigin),
                                    String.valueOf(searchRadiusDestination),
                                    String.valueOf(false)
                            });

                            return new OptimizationResult(poolingTimeWindow, searchRadiusOrigin, searchRadiusDestination, actualPerformance[3]);
                        }
                        return null;
                    }
                }));
            }

            for (Future<OptimizationResult> future : futures) {
                try {
                    OptimizationResult result = future.get(); // Wait for each task to complete
                    if (result != null) {
                        synchronized (this) {
                            addDataPoint(result.poolingTimeWindow, result.searchRadiusOrigin, result.searchRadiusDestination, result.performance);
                            if (result.performance > bestPerformance.get()) {
                                bestPerformance.set(result.performance);
                                bestParams.set(new int[]{result.poolingTimeWindow, result.searchRadiusOrigin, result.searchRadiusDestination});
                            }
                        }
                    }
                } catch (Exception e) {
                    System.err.println("Error in optimization thread: " + e.getMessage());
                    // Optionally, you might want to break the loop or take other actions here
                }
            }
        }

        executorService.shutdown();
        return bestParams.get();
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

        public int getRandomValue(Random rand) {
            return rand.nextInt(max - min + 1) + min;
        }
    }

    public static void main(String[] args) throws Exception {
        ParameterRange poolingTimeWindowRange = new ParameterRange(5, 15);
        ParameterRange searchRadiusOriginRange = new ParameterRange(0, 5000);
        ParameterRange searchRadiusDestinationRange = new ParameterRange(0, 5000);

        BayesianOptimization optimization = new BayesianOptimization(
                poolingTimeWindowRange, searchRadiusOriginRange, searchRadiusDestinationRange);

        int[] bestParams = optimization.optimizeParameters(100);
        System.out.println("Best Parameters: Pooling Time Window = " + bestParams[0] +
                ", Search Radius Origin = " + bestParams[1] +
                ", Search Radius Destination = " + bestParams[2]);
    }
}