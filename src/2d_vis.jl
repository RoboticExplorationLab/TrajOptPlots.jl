function circleShape(x, y, r)
    θ = range(0, 2π, length=100)
    x .+ r*sin.(θ), y .+ r*cos.(θ)
end

@recipe function f(con::CircleConstraint; circle_color=:red)
    p = length(con)
    for i = 1:p
        circ = circleShape(con.x[i], con.y[i], con.radius[i])
        @series begin
            aspect_ratio --> :equal
            seriestype := :shape
            label --> :none
            linecolor --> circle_color 
            linewidth --> 0.5
            fillalpha --> 1
            fillcolor --> circle_color 
            primary := false
            circ
        end
    end
    primary := false
    ()
end

@recipe function f(con::T) where T <: TrajectoryOptimization.AbstractConstraint
    primary := false
    ()
end

@recipe f(::Type{T}, conval::T) where T <: TrajectoryOptimization.ConVal = conval.con


@recipe function f(conSet::TrajectoryOptimization.AbstractConstraintSet)
    for con in conSet
        @series begin
            con
        end
    end
    aspect_ratio --> :equal
    primary := false
    ()
end


## Plotting trajectories for specific models
@recipe function f(prob::Problem) 
    @series begin
        prob.constraints 
    end
    (prob.model, prob.Z)
end

@recipe function f(car::RobotZoo.DubinsCar, Z::Traj)
    linecolor --> :black
    linewidth --> 2
    aspect_ratio --> :equal
    xind := 1
    yind := 2
    RobotDynamics.Traj2((Z,))
end

@recipe function f(model::RobotZoo.DoubleIntegrator, Z::Traj)
    label --> ["position" "velocity"]
    linewidth --> 2
    (RobotDynamics.get_times(Z), states(Z))
end

