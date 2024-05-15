package org.eqasim.sao_paulo.siting.ga;

import java.util.Arrays;
import java.util.Random;

public class GeneticAlgorithm {
    private static final int POP_SIZE = 100; // Population size
    private static final int MAX_GENERATIONS = 100; // Max number of generations
    private static final double MUTATION_RATE = 0.05; // Mutation rate
    private static final double CROSSOVER_RATE = 0.7; // Crossover rate
    private static final int TOURNAMENT_SIZE = 5; // Tournament size for selection
    private static final Random rand = new Random();

    private static final double ALPHA = 1.0; // Weight for saved flight distances
    private static final double BETA = 0.5; // Weight for additional travel time

    // Assuming these arrays are initialized elsewhere in your code:
    private static double[] flightDistances; // Distances for each trip
    private static double[][] accessTimesOriginal; // Original access times for each trip
    private static double[][] accessTimesUpdated; // Updated access times for each trip and vehicle
    private static double[][] egressTimesOriginal; // Original egress times for each trip
    private static double[][] egressTimesUpdated; // Updated egress times for each trip and vehicle
    private static double[][] waitingTimes; // Waiting times at the parking station for each trip and vehicle

    // Main method to run the GA
    public static void main(String[] args) {
        int[][] population = initializePopulation(POP_SIZE);
        for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
            population = evolvePopulation(population);
            System.out.println("Generation " + gen + ": Best fitness = " + findBestFitness(population));
        }
    }

    // Initialize population with random assignments
    private static int[][] initializePopulation(int size) {
        int[][] population = new int[size][];
        for (int i = 0; i < size; i++) {
            population[i] = generateIndividual();
        }
        return population;
    }

    // Generate a random individual
    private static int[] generateIndividual() {
        int[] individual = new int[NUMBER_OF_TRIPS]; // Assume NUMBER_OF_TRIPS is defined
        for (int i = 0; i < individual.length; i++) {
            individual[i] = rand.nextInt(NUMBER_OF_VEHICLES); // Assume NUMBER_OF_VEHICLES is defined
        }
        return individual;
    }

    // Evolve population
    private static int[][] evolvePopulation(int[][] pop) {
        int[][] newPop = new int[POP_SIZE][];
        for (int i = 0; i < POP_SIZE; i++) {
            int[] parent1 = select(pop);
            int[] parent2 = select(pop);
            int[] child = crossover(parent1, parent2);
            newPop[i] = mutate(child);
        }
        return newPop;
    }

    // Selection - Tournament selection
    private static int[] select(int[][] pop) {
        int[] best = null;
        double bestFitness = Double.MIN_VALUE;
        for (int i = 0; i < TOURNAMENT_SIZE; i++) {
            int[] individual = pop[rand.nextInt(POP_SIZE)];
            double fitness = calculateFitness(individual);
            if (fitness > bestFitness) {
                best = individual;
                bestFitness = fitness;
            }
        }
        return Arrays.copyOf(best, best.length);
    }

    // Crossover - Single point crossover
    private static int[] crossover(int[] parent1, int[] parent2) {
        int[] child = new int[parent1.length];
        if (rand.nextDouble() < CROSSOVER_RATE) {
            int crossoverPoint = rand.nextInt(parent1.length);
            for (int i = 0; i < crossoverPoint; i++) {
                child[i] = parent1[i];
            }
            for (int i = crossoverPoint; i < parent2.length; i++) {
                child[i] = parent2[i];
            }
        } else {
            return rand.nextBoolean() ? parent1 : parent2;
        }
        return child;
    }

    // Mutation - Randomly change vehicle assignment
    private static int[] mutate(int[] individual) {
        for (int i = 0; i < individual.length; i++) {
            if (rand.nextDouble() < MUTATION_RATE) {
                individual[i] = rand.nextInt(NUMBER_OF_VEHICLES);
            }
        }
        return individual;
    }

    // Calculate fitness for an individual
    private static double calculateFitness(int[] individual) {
        double fitness = 0;
        int trips = individual.length; // Number of trips

        for (int i = 0; i < trips; i++) {
            int v = individual[i]; // Vehicle assigned to trip i
            double savedFlightDistance = flightDistances[i];
            double additionalTravelTime = accessTimesUpdated[i][v] - accessTimesOriginal[i][v]
                    + egressTimesUpdated[i][v] - egressTimesOriginal[i][v]
                    + waitingTimes[i][v];

            fitness += (ALPHA * savedFlightDistance - BETA * additionalTravelTime);
        }

        return fitness;
    }

    // Find the best fitness in the current population
    private static double findBestFitness(int[][] population) {
        double bestFitness = Double.MIN_VALUE;
        for (int[] individual : population) {
            double fitness = calculateFitness(individual);
            if (fitness > bestFitness) {
                bestFitness = fitness;
            }
        }
        return bestFitness;
    }
}
