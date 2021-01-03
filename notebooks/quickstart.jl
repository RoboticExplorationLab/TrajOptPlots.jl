using TrajOptPlots
using RobotZoo: Cartpole
using MeshCat
using StaticArrays

# Start visualizer
vis = Visualizer()
open(vis)

# Display the model
model = Cartpole()
TrajOptPlots.set_mesh!(vis, model)

# Visualize a single state
x = SA[0.5, pi/3, 0, 0]
visualize!(vis, model, x)

# Visualize a trajectory
tf = 2.0
X = [SA[sin(t), cos(t), 0, 0] for t in range(0, 2.0, length=21)]
visualize!(vis, model, tf, X)