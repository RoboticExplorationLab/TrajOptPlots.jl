module TrajOptPlots

using MeshCat
using GeometryTypes
using Rotations
using CoordinateTransformations
using TrajectoryOptimization
using Plots
using StaticArrays
using FileIO
using MeshIO
using RobotDynamics
using RobotZoo

import RobotDynamics: orientation

export
    visualize!,
    plot

include("rigid_bodies.jl")
include("line_plots.jl")
include("2d_vis.jl")
include("models.jl")


end # module
