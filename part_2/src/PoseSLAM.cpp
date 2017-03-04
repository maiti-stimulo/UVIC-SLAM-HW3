#include <fstream>
#include <string>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <boost/algorithm/string/split.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/algorithm/string/classification.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h> // read G20 files
using namespace std;
using namespace gtsam;
using namespace boost;

int main(int argc, char** argv) {
  // Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;
  // A prior factor consists of a mean and a noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  // Add a prior on the first pose, setting it to the origin
  graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));
  // For simplicity, we will use the same noise model for odometry and loop closures
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));
  // Create the data structure to hold the initial estimate to the solution
  Values initial;
  // Read from our manhattan dataset, in datasets/manhattan/Olson
  // creem una variable per definir el arxiu g2o
  string dataset = "../datasets/manhattan/Olson/manhattan.txt";
  //string dataset = "../datasets/manhattan/Olson/manhattanOlson3500.g2o";
  // Create variables to store the file nodes and values
  NonlinearFactorGraph::shared_ptr file_graph;
  Values::shared_ptr file_initial;
  // Get the values, in 2D mode
  tie(file_graph, file_initial) = readG2o(dataset, false);
  // Asign the values to the variales
  graph = *file_graph;// marquem un punter
  initial = *file_initial;// marquem un punter

  graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));
  // Loop over lines in dataset
  //// Insert vector values into the initial estimate data structure
  //// Insert edge values into the factor graph container

  // Assign the non optimized values from the LevenbergMarquardtOptimizer to a Values type
  LevenbergMarquardtOptimizer optimizer(graph,initial);
  Values non_optimized_result = optimizer.values();
  // Output non optimized graph to dot graph file
  ofstream non_optimized_dot_file("NonOptimizedGraph.dot");
  graph.saveGraph(non_optimized_dot_file, non_optimized_result);

  // Assign the optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type optimized_result
  Values optimized_result = optimizer.optimize();
  // Output optimized graph to dot graph file
  ofstream optimized_dot_file("OptimizedGraph.dot");
  graph.saveGraph(optimized_dot_file, optimized_result);

return 0;
}
