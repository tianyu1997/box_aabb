import numpy as np
from pydrake.solvers import MathematicalProgram, LinearCost, Binding, Cost
from pydrake.geometry.optimization import GraphOfConvexSets, Point
gcs = GraphOfConvexSets()
u = gcs.AddVertex(Point(np.zeros(2)), "u")
v = gcs.AddVertex(Point(np.zeros(2)), "v")
edge = gcs.AddEdge(u, v, "edge")
try:
    edge_cost = LinearCost(np.zeros(4))
    binding = Binding[Cost](edge_cost, np.append(u.x(), v.x()))
    ret = edge.AddCost(binding)
    print("AddCost returned:", type(ret), ret)
    print("Can it be subscripted?", hasattr(ret, "__getitem__"))
    if hasattr(ret, "__getitem__"):
        print("ret[1] =", ret[1])
except Exception as e:
    import traceback
    traceback.print_exc()
