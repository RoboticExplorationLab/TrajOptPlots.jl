module TrajOptPlots

using MeshCat
# using GeometryTypes
using GeometryBasics
using Rotations
using CoordinateTransformations
using TrajectoryOptimization
using RecipesBase 
using StaticArrays
using FileIO
using MeshIO
using RobotDynamics
using RobotZoo
using Colors

import RobotDynamics: orientation

export
    visualize!,
    waypoints!


include("meshes.jl")
include("rigid_bodies.jl")
include("2d_vis.jl")
include("models.jl")


end # module
