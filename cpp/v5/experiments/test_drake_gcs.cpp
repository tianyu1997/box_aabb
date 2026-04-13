// Minimal Drake GCS test to verify the library works
#include <iostream>
#include <Eigen/Dense>

#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/cost.h>

using drake::geometry::optimization::GraphOfConvexSets;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;

int main() {
    std::cerr << "=== Drake GCS Minimal Test ===" << std::endl;

    const int n = 2;  // 2D for simplicity
    
    // Step 1: Create GCS
    std::cerr << "Step 1: Creating GCS..." << std::endl;
    GraphOfConvexSets gcs;

    // Step 2: Add a single HPolyhedron vertex (box [0,1]^2)
    std::cerr << "Step 2: Adding box vertex..." << std::endl;
    Eigen::VectorXd lb(n), ub(n);
    lb << 0.0, 0.0;
    ub << 1.0, 1.0;
    auto hpoly = HPolyhedron::MakeBox(lb, ub);
    auto* v_box = gcs.AddVertex(hpoly, "box");
    std::cerr << "  box vertex added, x.size=" << v_box->x().size() << std::endl;

    // Step 3: Add start/goal point vertices
    std::cerr << "Step 3: Adding start/goal points..." << std::endl;
    Eigen::VectorXd start(n), goal(n);
    start << 0.1, 0.1;
    goal << 0.9, 0.9;
    auto* v_start = gcs.AddVertex(Point(start), "start");
    auto* v_goal  = gcs.AddVertex(Point(goal), "goal");
    std::cerr << "  start vertex x.size=" << v_start->x().size() << std::endl;
    std::cerr << "  goal vertex x.size=" << v_goal->x().size() << std::endl;

    // Step 4: Add edges
    std::cerr << "Step 4: Adding edges..." << std::endl;
    auto* e1 = gcs.AddEdge(v_start, v_box);
    auto* e2 = gcs.AddEdge(v_box, v_goal);
    std::cerr << "  edges added" << std::endl;

    // Step 5: Add cost using Binding<QuadraticCost>
    std::cerr << "Step 5: Adding costs via Binding<QuadraticCost>..." << std::endl;
    for (auto* edge : gcs.Edges()) {
        auto& xu = edge->xu();
        auto& xv = edge->xv();
        std::cerr << "  edge: xu.size=" << xu.size() << " xv.size=" << xv.size() << std::endl;

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
        std::cerr << "  cost added OK" << std::endl;
    }

    // Step 6: Solve
    std::cerr << "Step 6: Solving..." << std::endl;
    drake::geometry::optimization::GraphOfConvexSetsOptions opts;
    opts.convex_relaxation = true;
    auto result = gcs.SolveShortestPath(*v_start, *v_goal, opts);

    std::cerr << "  success=" << result.is_success() << std::endl;
    if (result.is_success()) {
        double cost_val = result.get_optimal_cost();
        std::cerr << "  cost=" << cost_val << std::endl;
        
        Eigen::VectorXd box_sol = result.GetSolution(v_box->x());
        std::cerr << "  box_solution=" << box_sol.transpose() << std::endl;
    }

    std::cerr << "=== Test Complete ===" << std::endl;
    return 0;
}
