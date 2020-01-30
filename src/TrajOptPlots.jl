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

import TrajectoryOptimization.Dynamics: orientation

export
    visualize!,
    plot

include("rigid_bodies.jl")

function Plots.plot(A::Vector{<:SVector{N}}, inds=1:N; kwargs...) where N
    n = length(inds)
    X = zeros(n,length(A))
    for k in eachindex(A)
        X[:,k] = A[k][inds]
    end
    plot(X'; kwargs...)
end

end # module
