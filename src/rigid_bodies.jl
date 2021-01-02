# import TrajectoryOptimization.Controllers: RBState

using TrajectoryOptimization: get_trajectory, get_model, AbstractTrajectory


visualize!(vis, solver) = visualize!(vis, get_model(solver), get_trajectory(solver))

"""
    visualize!(vis, model, x)

Visualize a single configuration.
Defaults to using the `position` and `orientation` methods on `model`.

Overload this method to specify the visualization for a specific model.
"""
function visualize!(vis, model::AbstractModel, x::StaticVector, addrobot::Bool=true)
    visualize!(vis, RBState(model, x), addrobot)
end

""" Set state to RBState """
function visualize!(vis, x::RBState{<:Real}, addrobot::Bool=true)
    p = position(x)
    q = orientation(x)
    robot = addrobot ? vis["robot"] : vis
    settransform!(robot, compose(Translation(p), LinearMap(UnitQuaternion(q))))
end

"""
Animate the trajectory of a rigid body in MeshCat
    Assumes the robot geometry is already loaded into `vis["robot"]`
"""
visualize!(vis, model::AbstractModel, Z::AbstractTrajectory) =
    visualize!(vis, model, Z[end].t, states(Z))

"""
    visualize!(vis, probs...; kwargs...)
    visualize!(vis, model, Zs...; kwargs...)
    visualize!(vis, model, tf, Xs...; kwargs...)

Visualize many different trajectories of the same model.

# Arguments
* `probs`: any struct that supports `TO.get_model` and `TO.get_trajectory`
* `Zs`: any `AbstractTrajectory`
* `Xs`: a vector of state vectors
* `colors` (optional): Can be either `nothing` (default), a `Colorant`, or a vector of both, of equal length as the number of trajectories. Changes the color of the models in the order provided.
"""
function visualize!(vis, probs...; kwargs...)
    model = get_model(probs[1])
    visualize!(vis, model, get_trajectory.(probs)...; kwargs...)
end
function visualize!(vis, model::AbstractModel, Zs::Vararg{<:AbstractTrajectory}; kwargs...)
    tf = Zs[1][end].t - Zs[1][1].t
    visualize!(vis, model, tf, states.(Zs)...; kwargs...)
end
function visualize!(vis, model::AbstractModel, tf::Real, Xs...; colors=nothing)
    N = length(Xs[1])
    fps = Int(floor((N-1)/tf))
    anim = MeshCat.Animation(fps)
    num_traj = length(Xs)
    if isnothing(colors)
        colors = fill(nothing, length(Xs))
    elseif colors isa Colorant
        colors = fill(colors, length(Xs)-1)
        colors = [nothing; colors]
    elseif colors isa Vector{<:Union{Nothing, Colorant}}
        @assert length(colors) == length(Xs) "Number of colors must match the number of trajectories"
        set_mesh!(vis, model, color=colors[1])
    end
    for i = 2:num_traj
        TrajOptPlots.set_mesh!(vis["robot_copies/robot$i"], model, color=colors[i])
    end
    for k = 1:N
        atframe(anim, k) do
            for i = 1:num_traj
                if i == 1
                    robot = vis
                else
                    robot = vis["robot_copies"]["robot$i"]
                end
                visualize!(robot, model, Xs[i][k]) 
            end
        end
    end
    setanimation!(vis, anim)
end


#--- Environment plotting
function plot_cylinder(vis,c1,c2,radius,mat,name="")
    geom = Cylinder(Point3f0(c1),Point3f0(c2),convert(Float32,radius))
    setobject!(vis["cyl"][name],geom,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
end

function add_cylinders!(vis,x,y,r; height=1.5, idx=0, robot_radius=0.0, color=colorant"red")
    for i in eachindex(x)
        obj = Cylinder(Point3f0(x[i],y[i],0.0), Point3f0(x[i],y[i],height),
            Float32(r[i] - robot_radius))
        mat = MeshPhongMaterial(color=color)
        j = i + idx
        setobject!(vis["obs"]["cyl$j"], obj, mat)
    end
end

function add_cylinders!(vis,solver; kwargs...)
    conSet = TrajectoryOptimization.get_constraints(solver)
    idx = 0
    for con in conSet.constraints
        if con isa CircleConstraint
            add_cylinders!(vis, con.x, con.y, con.radius, idx=idx; kwargs...)
            idx += length(con)
        end
    end
end

function add_point!(vis, x::AbstractVector;
        radius=0.1, color=colorant"green", name="point")
    setobject!(vis[name], HyperSphere(Point3f0(x[1], x[2], x[3]), Float32(radius)),
        MeshPhongMaterial(color=color))
end



#--- Waypoints
const TO = TrajectoryOptimization
waypoints!(vis, solver; kwargs...) = 
    waypoints!(vis, TO.get_model(solver), TO.get_trajectory(solver); kwargs...)
function waypoints!(vis, model::AbstractModel, Z::AbstractTrajectory; length=0, inds=Int[], 
        color=nothing, color_end=nothing)
    N = size(Z,1)
    if length > 0 && isempty(inds)
        inds = Int.(round.(range(1,N,length=length)))
    elseif !isempty(inds) && length == 0
        length = size(inds,1)
    else
        throw(ArgumentError("Have to pass either length or inds, but not both"))
    end
    if !isnothing(color)
        if isnothing(color_end)
            color_end = color
        end
        colors = range(color, color_end, length=size(inds,1))
        # set_mesh!(vis, model, color=colors[end])
    end

    delete!(vis["waypoints"])
    for (j,i) in enumerate(inds)
        if isnothing(color)
            set_mesh!(vis["waypoints"]["point$i"], model)
        else
            set_mesh!(vis["waypoints"]["point$i"], model, color=colors[j])
        end
            
        # settransform!(vis["waypoints"]["point$i"]["geom"], compose(Translation(0,0,0.07),LinearMap( RotY(pi/2)*RotZ(-pi/2) )))
        visualize!(vis["waypoints"]["point$i"], model, state(Z[i]))
    end
    visualize!(vis, model, state(Z[end]))
end

clear_waypoints!(vis) = delete!(vis["waypoints"])
clear_copies!(vis) = delete!(vis["robot_copies"])
