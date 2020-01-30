
# Convert between TrajOpt UnitQuaternion and Rotations.jl Quat
export
    Quat,
    set_mesh!,
    add_cylinders!,
    waypoints!,
    clear_waypoints!

Rotations.Quat(q::UnitQuaternion) = Quat(q.s, q.x, q.y, q.z)
TrajectoryOptimization.UnitQuaternion(q::Rotations.Quat) = UnitQuaternion(q.w, q.x, q.y, q.z)

function plot_cylinder(vis,c1,c2,radius,mat,name="")
    geom = Cylinder(Point3f0(c1),Point3f0(c2),convert(Float32,radius))
    setobject!(vis["cyl"][name],geom,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
end

visualize!(vis, solver::TrajectoryOptimization.AbstractSolver) =
    visualize!(vis, get_model(solver), get_trajectory(solver))

"""
Animate the trajectory of a rigid body in MeshCat
    Assumes the robot geometry is already loaded into `vis["robot"]`
"""
function visualize!(vis, model::AbstractModel, Z::Traj)
    X = states(Z)
    fps = Int(floor(length(Z)/Z[end].t))
    anim = MeshCat.Animation(fps)
    for k in eachindex(Z)
        atframe(anim, k) do
            x = position(model, X[k])
            r = Dynamics.orientation(model, X[k])
            q = UnitQuaternion(Dynamics.orientation(model, X[k]))
            settransform!(vis["robot"], compose(Translation(x), LinearMap(Quat(q))))
        end
    end
    setanimation!(vis, anim)
    return anim
end

function add_cylinders!(vis,x,y,r; height=1.5, idx=0, robot_radius=0.0)
    for i in eachindex(x)
        obj = Cylinder(Point3f0(x[i],y[i],0.0), Point3f0(x[i],y[i],height),
            Float32(r[i] - robot_radius))
        mat = MeshPhongMaterial(color=RGBA(1,0,0,1.0))
        j = i + idx
        setobject!(vis["obs"]["cyl$j"], obj, mat)
    end
end

function add_cylinders!(vis,solver::TrajectoryOptimization.AbstractSolver; kwargs...)
    conSet = get_constraints(solver)
    idx = 0
    for conVal in conSet.constraints
        if conVal.con isa CircleConstraint
            con = conVal.con
            add_cylinders!(vis, con.x, con.y, con.radius, idx=idx; kwargs...)
            idx += length(con)
        end
    end
end

function get_mesh!(model::Dynamics.Quadrotor2)
    traj_folder = joinpath(dirname(pathof(TrajectoryOptimization)),"..")
    urdf_folder = joinpath(traj_folder, "dynamics","urdf")
    obj = joinpath(urdf_folder, "quadrotor_base.obj")
    quad_scaling = 0.085
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling
    return robot_obj, MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0))
end

get_mesh!(::Dynamics.Satellite2; dims=[1,1,2]*0.5) = HyperRectangle(Vec((-dims/2)...), Vec(dims...))
get_mesh!(model::InfeasibleModel) = get_mesh!(model.model)

function set_mesh!(vis, model::AbstractModel; kwargs...)
    setobject!(vis["robot"]["geom"], get_mesh!(model; kwargs...)...)
end

function get_mesh!(::Dynamics.YakPlane)
    meshfile = joinpath(@__DIR__,"..","data","meshes","piper","piper_pa18.obj")
    jpg = joinpath(@__DIR__,"..","data","meshes","piper","piper_diffuse.jpg")
    img = PngImage(jpg)
    texture = Texture(image=img)
    mat = MeshLambertMaterial(map=texture)
    obj = FileIO.load(meshfile)
    scaling = 0.05
    obj.vertices .= obj.vertices .* scaling
    return obj, mat
end

function set_mesh!(vis, model::Dynamics.YakPlane)
    obj,mat = get_mesh!(model)
    setobject!(vis["robot"]["geom"], obj, mat)
    settransform!(vis["robot"]["geom"], compose(Translation(0,0,0.07),LinearMap( RotY(pi/2)*RotZ(-pi/2) )))
end


function visualize!(vis, model::Dynamics.Quadrotor, Z::Traj)

    X = states(Z)
    fps = Int(round(length(Z)/Z[end].t))
    anim = MeshCat.Animation(fps)
    for k in eachindex(Z)
        atframe(anim, k) do
            x = X[k]
            r = @SVector [x[1], x[2], x[3]]
            q = UnitQuaternion(x[4], x[5], x[6], x[7])
            settransform!(vis["robot"], compose(Translation(r), LinearMap(Quat(q))))
        end
    end
    setanimation!(vis, anim)
    return anim
end

function waypoints!(vis, model::AbstractModel, Z::Traj; length=0, inds=Int[])
    N = size(Z,1)
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
        setobject!(vis["waypoints"]["point$i"], obj, mat)
        x = state(Z[i])
        r = position(model, x)
        q = UnitQuaternion(orientation(model, x))
        settransform!(vis["waypoints"]["point$i"], compose(Translation(r), LinearMap(Quat(q))))
    end
end

clear_waypoints!(vis) = delete!(vis["waypoints"])
