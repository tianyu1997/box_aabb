// Test Drake GCS with a larger graph (chain of boxes) to reproduce crash
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/cost.h>

using drake::geometry::optimization::GraphOfConvexSets;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;

int main() {
    const int n = 7;         // 7-DOF like KUKA
    const int n_boxes = 127; // match the corridor size that crashes

    std::cerr << "=== Drake GCS Large Test ===" << std::endl;
    std::cerr << "n=" << n << " n_boxes=" << n_boxes << std::endl;

    GraphOfConvexSets gcs;
    std::vector<GraphOfConvexSets::Vertex*> verts;

    // Add chain of boxes
    for (int i = 0; i < n_boxes; ++i) {
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(n, -1.0 + 0.01 * i);
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(n,  1.0 + 0.01 * i);
        auto hpoly = HPolyhedron::MakeBox(lb, ub);
        verts.push_back(gcs.AddVertex(hpoly, "box_" + std::to_string(i)));
    }

    // Start/Goal
    Eigen::VectorXd start = Eigen::VectorXd::Constant(n, -0.5);
    Eigen::VectorXd goal  = Eigen::VectorXd::Constant(n,  0.5 + 0.01 * n_boxes);
    auto* v_start = gcs.AddVertex(Point(start), "start");
    auto* v_goal  = gcs.AddVertex(Point(goal),  "goal");
    gcs.AddEdge(v_start, verts[0]);
    gcs.AddEdge(verts.back(), v_goal);

    // Chain edges (bidirectional) and some skip edges
    for (int i = 0; i + 1 < n_boxes; ++i) {
        gcs.AddEdge(verts[i], verts[i+1]);
        gcs.AddEdge(verts[i+1], verts[i]);
    }
    // Add some skip edges for non-trivial adjacency
    for (int i = 0; i + 2 < n_boxes; i += 3) {
        gcs.AddEdge(verts[i], verts[i+2]);
        gcs.AddEdge(verts[i+2], verts[i]);
    }

    std::cerr << "Vertices: " << gcs.Vertices().size()
              << " Edges: " << gcs.Edges().size() << std::endl;

    // Add costs
    std::cerr << "Adding costs..." << std::endl;
    int n_cost = 0;
    for (auto* edge : gcs.Edges()) {
        auto& xu = edge->xu();
        auto& xv = edge->xv();

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2*n, 2*n);
        Q.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n);
        Q.topRightCorner(n, n) = -Eigen::MatrixXd::Identity(n, n);
        Q.bottomLeftCorner(n, n) = -Eigen::MatrixXd::Identity(n, n);
        Q.bottomRightCorner(n, n) = Eigen::MatrixXd::Identity(n, n);

        Eigen::VectorXd b = Eigen::VectorXd::Zero(2*n);

        drake::solvers::VectorXDecisionVariable vars(2*n);
        for (int d = 0; d < n; ++d) vars[d] = xu[d];
        for (int d = 0; d < n; ++d) vars[n+d] = xv[d];

        auto cost = std::make_shared<drake::solvers::QuadraticCost>(Q, b, 0.0);
        edge->AddCost(drake::solvers::Binding<drake::solvers::Cost>(cost, vars));
        ++n_cost;
    }
    std::cerr << "Costs added: " << n_cost << std::endl;

    // Solve
    std::cerr << "Solving..." << std::endl;
    drake::geometry::optimization::GraphOfConvexSetsOptions opts;
    opts.convex_relaxation = true;
    auto result = gcs.SolveShortestPath(*v_start, *v_goal, opts);

    std::cerr << "success=" << result.is_success() << std::endl;
    if (result.is_success()) {
        std::cerr << "cost=" << result.get_optimal_cost() << std::endl;
    }

    std::cerr << "=== Done ===" << std::endl;
    return 0;
}
