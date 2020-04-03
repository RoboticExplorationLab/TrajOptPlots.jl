function circleShape(x, y, r)
    θ = range(0, 2π, length=100)
    x .+ r*sin.(θ), y .+ r*cos.(θ)
end

function Plots.plot!(con::CircleConstraint{T,P}) where {T,P}
    for i = 1:P
        circ = circleShape(con.x[i], con.y[i], con.radius[i])
        plot!(circ, seriestype=[:shape,], lw=0.5, c=:red, linecolor=:red, legend=false,
            aspect_ratio = :equal)
    end
end

@inline Plots.plot!(plot::TrajectoryOptimization.AbstractConstraint) = nothing
@inline Plots.plot!(con::TrajOptCore.ConVal) = plot!(con.con)

function Plots.plot!(conSet::TrajOptCore.AbstractConstraintSet)
    for con in conSet
        plot!(con)
    end
end

function Plots.plot(solver::TrajectoryOptimization.AbstractSolver)
    p = plot()
    plot!(get_constraints(solver))
    plot!(get_model(solver), get_trajectory(solver))
    p
end

@inline Plots.plot!(model::TrajectoryOptimization.InfeasibleModel, Z::Traj) = plot!(model.model, Z)
function Plots.plot!(car::RobotZoo.DubinsCar, Z::Traj)
    plot_trajectory!(states(Z), c=:black, lw=2.0)
end

@inline Plots.plot!(model::RobotZoo.DoubleIntegrator, Z::Traj) = plot!(model, states(Z))
function Plots.plot!(model::RobotZoo.DoubleIntegrator, X)
    plot!(Z, lw=2.0, labels=["position" "velocity"])
end

function plot_trajectory!(X::Vector{<:AbstractVector})
    x = [x[1] for x in X]
    y = [x[2] for x in X]
    plot(x,y, aspect_ratio=:equal)
end
