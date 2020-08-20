# import TrajectoryOptimization.Controllers: RBState

using TrajectoryOptimization: get_trajectory, get_model, AbstractTrajectory


visualize!(vis, solver) = visualize!(vis, get_model(solver), get_trajectory(solver))

"""
    visualize!(vis, model, x)

Visualize a single configuration.
Defaults to using the `position` and `orientation` methods on `model`.

Overload this method to specify the visualization for a specific model.
"""
function visualize!(vis, model::AbstractModel, x::AbstractVector, addrobot::Bool=true)
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
Visualize many different trajectories of the same model
"""
function visualize!(vis, probs...)
    model = get_model(probs[1])
    visualize!(vis, model, get_trajectory.(probs)...)
end
function visualize!(vis, model::AbstractModel, Zs::Vararg{<:AbstractTrajectory})
    tf = Zs[1][end].t
    visualize!(vis, model, tf, states.(Zs)...)
end
function visualize!(vis, model::AbstractModel, tf::Real, Xs...)
    N = length(Xs[1])
    fps = Int(floor(N/tf))
    anim = MeshCat.Animation(fps)
    num_traj = length(Xs)
    for i = 2:num_traj
        TrajOptPlots.set_mesh!(vis["robot_copies/robot$i"], model)
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
function waypoints!(vis, model::AbstractModel, Z::AbstractTrajectory; length=0, inds=Int[])
    N = size(Z,1)
    if length > 0 && isempty(inds)
        inds = Int.(round.(range(1,N,length=length)))
    elseif !isempty(inds) && length == 0
        length = size(inds,1)
    else
        throw(ArgumentError("Have to pass either length or inds, but not both"))
    end
    delete!(vis["waypoints"])
    for (j,i) in enumerate(inds)
        set_mesh!(vis["waypoints"]["point$i"], model)
        # settransform!(vis["waypoints"]["point$i"]["geom"], compose(Translation(0,0,0.07),LinearMap( RotY(pi/2)*RotZ(-pi/2) )))
        visualize!(vis["waypoints"]["point$i"], model, state(Z[i]))
    end
end

function waypoints!(vis, model, Xs...; length=0, inds=Int[])
    colors = [RGBA(0.0,0,0,1.0), colorant"cyan2", colorant"darkorange"]
    X = Xs[1]
    N = size(X,1)
    if length > 0 && isempty(inds)
        inds = Int.(round.(range(1,N,length=length)))
    elseif !isempty(inds) && length == 0
        length = size(inds,1)
    else
        throw(ArgumentError("Have to pass either length or inds, but not both"))
    end
    obj,mat = get_mesh!(model)
    delete!(vis["waypoints"])
    for i in inds
        for (j,X) in enumerate(Xs)
            setobject!(vis["waypoints"]["robot$j"]["point$i"]["geom"],
                obj, MeshPhongMaterial(color=colors[(j-1%3)+1]))
            x = X[i]
            r = x.r
            q = x.q
            settransform!(vis["waypoints"]["robot$j"]["point$i"],
                compose(Translation(r), LinearMap(UnitQuaternion(q))))
        end
    end
end

clear_waypoints!(vis) = delete!(vis["waypoints"])
