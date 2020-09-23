using FileIO

function set_mesh!(vis, model::AbstractModel; kwargs...)
    _set_mesh!(vis["robot"], model; kwargs...)
end

RobotDynamics.RBState(model::AbstractModel, x) = 
    RBState(position(model, x), orientation(model, x), zeros(3), zeros(3))

function _set_mesh!(vis, model::RobotZoo.DoubleIntegrator{<:Any,2})
    radius = 0.1f0
    body = Cylinder(Point3f0(0,0,0), Point3f0(0,0,0.05), radius)
    setobject!(vis["geom"]["body"], body, MeshPhongMaterial(color=colorant"green"))
end

# Pendulum
function _set_mesh!(vis, model::RobotZoo.Pendulum)
    hinge = Cylinder(Point3f0(0.05,0,0), Point3f0(-0.05,0,0), 0.05f0)
    rod   = Cylinder(Point3f0(0,0,0), Point3f0(0,0,model.length), 0.01f0)
    mass  = HyperSphere(Point3f0(0,0,model.length), 0.05f0)
    setobject!(vis["geom"]["hinge"], hinge, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["geom"]["rod"  ], rod,   MeshPhongMaterial(color=colorant"blue"))
    setobject!(vis["geom"]["mass" ], mass , MeshPhongMaterial(color=colorant"red"))
end

# Cartpole
function _set_mesh!(vis, model::RobotZoo.Cartpole)
    dim = Vec(0.1, 0.3, 0.1)
    rod = Cylinder(Point3f0(0,-10,0), Point3f0(0,10,0), 0.01f0)
    cart = HyperRectangle(-dim/2, dim)
    hinge = Cylinder(Point3f0(-dim[1]/2,0,dim[3]/2), Point3f0(dim[1],0,dim[3]/2), 0.03f0)

    pole = Cylinder(Point3f0(0,0,0),Point3f0(0,0,model.l),0.01f0)
    mass = HyperSphere(Point3f0(0,0,model.l), 0.05f0)
    setobject!(vis["rod"], rod, MeshPhongMaterial(color=colorant"grey"))
    setobject!(vis["cart","box"],   cart, MeshPhongMaterial(color=colorant"green"))
    setobject!(vis["cart","hinge"], hinge, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["cart","pole","geom","cyl"], pole, MeshPhongMaterial(color=colorant"blue"))
    setobject!(vis["cart","pole","geom","mass"], mass, MeshPhongMaterial(color=colorant"red"))
    settransform!(vis["cart","pole"], Translation(0.75*dim[1],0,dim[3]/2))
end

function visualize!(vis, model::RobotZoo.Cartpole, x::StaticVector)
    y = x[1]
    θ = x[2]
    q = expm((pi-θ) * @SVector [1,0,0])
    settransform!(vis["robot","cart"], Translation(0,-y,0))
    settransform!(vis["robot","cart","pole","geom"], LinearMap(UnitQuaternion(q)))
end

# Acrobot (doublependulum)
function _set_mesh!(vis, model::RobotZoo.Acrobot)
    hinge = Cylinder(Point3f0(-0.05,0,0), Point3f0(0.05,0,0), 0.05f0)
    thick = 0.05
    dim1  = Vec(thick, thick, model.l[1])
    link1 = HyperRectangle(Vec(-thick/2,-thick/2,0),dim1)
    dim2  = Vec(thick, thick, model.l[2])
    link2 = HyperRectangle(Vec(-thick/2,-thick/2,0),dim2)
    setobject!(vis["base"], hinge, MeshPhongMaterial(color=colorant"grey"))
    setobject!(vis["link1"], link1, MeshPhongMaterial(color=colorant"blue"))
    setobject!(vis["link1","joint"], hinge, MeshPhongMaterial(color=colorant"grey"))
    setobject!(vis["link1","link2"], link2, MeshPhongMaterial(color=colorant"blue"))
    settransform!(vis["link1","link2"], Translation(0,0,model.l[2]))
    settransform!(vis["link1","joint"], Translation(0,0,model.l[2]))
end

function visualize!(vis, model::RobotZoo.Acrobot, x::StaticVector)
    e1 = @SVector [1,0,0]
    q1,q2 = expm((x[1]-pi/2)*e1), expm(x[2]*e1)
    settransform!(vis["robot","link1"], LinearMap(UnitQuaternion(q1)))
    settransform!(vis["robot","link1","link2"], compose(Translation(0,0,model.l[1]), LinearMap(UnitQuaternion(q2))))
end

# Dubins Car
function _set_mesh!(vis, model::RobotZoo.DubinsCar)
    radius = Float32(model.radius)
    body = Cylinder(Point3f0(0,0,0), Point3f0(0,0,0.05), radius)
    face = HyperRectangle(Vec(3radius/4, -radius/2, 0), Vec(radius/4, radius, 0.06))
    setobject!(vis["geom"]["body"], body, MeshPhongMaterial(color=colorant"blue"))
    setobject!(vis["geom"]["face"], face, MeshPhongMaterial(color=colorant"yellow"))
end

# Quadrotor
function _set_mesh!(vis, model::RobotZoo.Quadrotor; scaling=1.0)
    urdf_folder = joinpath(@__DIR__, "..", "data", "meshes")
    obj = joinpath(urdf_folder, "quadrotor_base.obj")
    quad_scaling = 0.085 * scaling
    # quad_scaling = 0.15
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling
    mat = MeshPhongMaterial(color=colorant"black")
    setobject!(vis["geom"], robot_obj, mat)
end

# Yak Plane
function _set_mesh!(vis, ::RobotZoo.YakPlane)
    # meshfile = joinpath(@__DIR__,"..","data","meshes","piper","piper_pa18.obj")
    meshfile = joinpath(@__DIR__,"..","data","meshes","cirrus","Cirrus.obj")
    jpg = joinpath(@__DIR__,"..","data","meshes","piper","piper_diffuse.jpg")
    img = PngImage(jpg)
    texture = Texture(image=img)
    mat = MeshLambertMaterial(map=texture)
    obj = FileIO.load(meshfile)
    scaling = 0.05
    obj.vertices .= obj.vertices .* scaling
    setobject!(vis["geom"], obj, mat)
    settransform!(vis["geom"], compose(Translation(0,0,0.07),LinearMap( RotY(pi/2)*RotZ(-pi/2) )))
end

# _set_mesh!(vis, model::TrajectoryOptimization.InfeasibleModel) = _set_mesh!(vis, model.model)

function _set_mesh!(vis, ::RobotZoo.Satellite; dims=[1,1,2]*0.5, color=colorant"grey70")
    obj = HyperRectangle(Vec((-dims/2)...), Vec(dims...))
    mat = MeshPhongMaterial(; color=color)
    setobject!(vis["geom"], obj, mat)
end

function _set_mesh!(vis, model::RigidBody; dims=[1,1,2]*0.5, color=colorant"grey70")
    obj = HyperRectangle(Vec((-dims/2)...), Vec(dims...))
    mat = MeshPhongMaterial(; color=color)
    setobject!(vis["geom"], obj, mat)
end


