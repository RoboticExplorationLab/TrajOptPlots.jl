using TrajOptPlots
using Test
using StaticArrays
# using RecipesBase
using Plots
using RobotDynamics
using TrajectoryOptimization
using RobotZoo
# using Altro
using MeshCat
using Rotations
using LinearAlgebra
const TO = TrajectoryOptimization

include("problems.jl")

@testset "2D Plots" begin
    include("2d_plots.jl")
end

@testset "3D Visualization" begin
    include("3d_vis.jl")
    include("models.jl")
end
