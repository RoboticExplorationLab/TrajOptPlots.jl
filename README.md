# TrajOptPlots.jl
This package provides methods for visualizing 2D and 3D systems, and is part of the [TrajectoryOptimization.jl](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl)
ecosystem. This package depends heavily upon [MeshCat.jl](https://github.com/rdeits/MeshCat.jl), which is used as the visualization backend. This package 
is basically a simple wrapper around MeshCat, providing convenient methods for the types defined in 
[RobotDynamics.jl](https://github.com/RoboticExplorationLab/RobotDynamics.jl) and
[TrajectoryOptimization.jl](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl). 

## Installation
To install, use the Julia package manager:
```julia
julia> ] # activate package manager
(@v1.5) pkg> add TrajOptPlots
```

We also recommend using the package manager to add the following packages to your environment:
* [MeshCat.jl](https://github.com/rdeits/MeshCat.jl): Provides visualization
* [RobotDynamics.jl](https://github.com/RoboticExplorationLab/RobotDynamics.jl): Defines dynamical systems
* [RobotZoo.jl](https://github.com/RoboticExplorationLab/RobotZoo.jl): Provides a handful of canoncial dynamical systems. All of these sytems can be visualized using TrajOptPlots
* [TrajectoryOptimization.jl](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl): Defines trajectory optimization problems
* [Altro.jl](https://github.com/RoboticExplorationLab/Altro.jl): Fast solver for trajectory optimization problems
* [StaticArrays.jl](https://github.com/JuliaArrays/StaticArrays.jl): Fast stack-allocated arrays

## Quick Start
Basic usage will usually follow something like this:
```julia
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
```

## Examples
For more detailed examples, see the [notebooks](https://github.com/RoboticExplorationLab/TrajOptPlots.jl/tree/master/notebooks)
