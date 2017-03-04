#include <fstream>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;
  // A prior factor consists of a mean and a noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));
  // Add a prior on the first pose, setting it to the origin
  graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));
  // For simplicity, we will use the same noise model for odometry and loop closures
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  // Create the data structure to hold the initial estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initial;
  //Insert the following initial estimate values into the initial estimate datastructure
  // Use initial.insert()
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initial.insert(4, Pose2(4.0, 2.0, M_PI));
  initial.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  // Insert the following odometry values, graph edges, into the factor graph container
  // Use graph.add()
  // Describing the edge with a BetweenFactor<Pose2>() object
   graph.add(BetweenFactor<Pose2>(1,2, Pose2(2, 0, 0), model));
   graph.add(BetweenFactor<Pose2>(2,3, Pose2(2, 0, M_PI_2), model));
   graph.add(BetweenFactor<Pose2>(3,4, Pose2(2, 0, M_PI_2), model));
   graph.add(BetweenFactor<Pose2>(4,5, Pose2(2, 0, M_PI_2), model));
   // Add loop closure constraint
   graph.add(BetweenFactor<Pose2>(5,2, Pose2(1.98, -0.06, 1.6), model)); // ha d'anar del 5 al 2, ja que si anés de 2 al 5 no pot ser ja que el 5 no existiria

  // Assign the non optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type non_optimized_result
  LevenbergMarquardtOptimizer optimizer(graph,initial);
  Values non_optimized_result = optimizer.values();
  // Output non optimized graph to dot graph file
  ofstream non_optimized_dot_file("NonOptimizedGraph.dot");
  graph.saveGraph(non_optimized_dot_file, non_optimized_result);
  // Marginal covariances for non optimized results
    cout.precision(3);
    Marginals non_optimized_marginals(graph,non_optimized_result);

    cout << "Marginal covariaces for non optimized result\n";
    cout << "x1 covariance:\n" << non_optimized_marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << non_optimized_marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << non_optimized_marginals.marginalCovariance(3) << endl;
    cout << "x4 covariance:\n" << non_optimized_marginals.marginalCovariance(4) << endl;
    cout << "x5 covariance:\n" << non_optimized_marginals.marginalCovariance(5) << endl;
  // Assign the optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type optimized_result
  Values optimized_result = optimizer.optimize();
  // Output optimized graph to dot graph file
  ofstream optimized_dot_file("OptimizedGraph.dot");
  graph.saveGraph(optimized_dot_file, optimized_result);

  Marginals optimized_marginals(graph,optimized_result);

    cout << "Marginal covariaces for optimized result\n";
    cout << "x1 covariance:\n" << optimized_marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << optimized_marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << optimized_marginals.marginalCovariance(3) << endl;
    cout << "x4 covariance:\n" << optimized_marginals.marginalCovariance(4) << endl;
    cout << "x5 covariance:\n" << optimized_marginals.marginalCovariance(5) << endl;
  return 0;
}
