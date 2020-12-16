using FileIO

function defcolor(c1, c2, c1def, c2def)
    if !isnothing(c1) && isnothing(c2)
        c2 = c1
    else
        c1 = isnothing(c1) ? c1def : c1
        c2 = isnothing(c2) ? c2def : c2
    end
    c1,c2
end

function set_mesh!(vis, model::AbstractModel; kwargs...)
    _set_mesh!(vis["robot"], model; kwargs...)
end

RobotDynamics.RBState(model::AbstractModel, x) = 
    RBState(position(model, x), orientation(model, x), zeros(3), zeros(3))

function _set_mesh!(vis, model::RobotZoo.DoubleIntegrator{<:Any,2}; 
        color=colorant"green", radius=0.1, height=0.05)
    radius = Float32(radius) 
    body = Cylinder(Point3f0(0,0,0), Point3f0(0,0,height), radius)
    setobject!(vis["geom"]["body"], body, MeshPhongMaterial(color=color))
end

# Pendulum
function _set_mesh!(vis, model::RobotZoo.Pendulum;
        color=nothing, color2=nothing, length=model.length)
    hinge = Cylinder(Point3f0(0.05,0,0), Point3f0(-0.05,0,0), 0.05f0)
    rod   = Cylinder(Point3f0(0,0,0), Point3f0(0,0,length), 0.01f0)
    mass  = HyperSphere(Point3f0(0,0,length), 0.05f0)
    c1,c2 = defcolor(color, color2, colorant"blue", colorant"red")
    setobject!(vis["geom"]["hinge"], hinge, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["geom"]["rod"  ], rod,   MeshPhongMaterial(color=c1))
    setobject!(vis["geom"]["mass" ], mass , MeshPhongMaterial(color=c2))
end

# Cartpole
function _set_mesh!(vis, model::RobotZoo.Cartpole; 
        color=nothing, color2=nothing)
    dim = Vec(0.1, 0.3, 0.1)
    rod = Cylinder(Point3f0(0,-10,0), Point3f0(0,10,0), 0.01f0)
    cart = Rect3D(-dim/2, dim)
    hinge = Cylinder(Point3f0(-dim[1]/2,0,dim[3]/2), Point3f0(dim[1],0,dim[3]/2), 0.03f0)
    c1,c2 = defcolor(color,color2, colorant"blue", colorant"red")

    pole = Cylinder(Point3f0(0,0,0),Point3f0(0,0,model.l),0.01f0)
    mass = HyperSphere(Point3f0(0,0,model.l), 0.05f0)
    setobject!(vis["rod"], rod, MeshPhongMaterial(color=colorant"grey"))
    setobject!(vis["cart","box"],   cart, MeshPhongMaterial(color=isnothing(color) ? colorant"green" : color))
    setobject!(vis["cart","hinge"], hinge, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["cart","pole","geom","cyl"], pole, MeshPhongMaterial(color=c1))
    setobject!(vis["cart","pole","geom","mass"], mass, MeshPhongMaterial(color=c2))
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
function _set_mesh!(vis, model::RobotZoo.Acrobot; color=colorant"blue", thick=0.05)
    hinge = Cylinder(Point3f0(-0.05,0,0), Point3f0(0.05,0,0), 0.05f0)
    dim1  = Vec(thick, thick, model.l[1])
    link1 = Rect3D(Vec(-thick/2,-thick/2,0),dim1)
    dim2  = Vec(thick, thick, model.l[2])
    link2 = Rect3D(Vec(-thick/2,-thick/2,0),dim2)
    mat1 = MeshPhongMaterial(color=colorant"grey")
    mat2 = MeshPhongMaterial(color=color)
    setobject!(vis["base"], hinge, mat1) 
    setobject!(vis["link1"], link1, mat2) 
    setobject!(vis["link1","joint"], hinge, mat1) 
    setobject!(vis["link1","link2"], link2, mat2) 
    settransform!(vis["link1","link2"], Translation(0,0,model.l[1]))
    settransform!(vis["link1","joint"], Translation(0,0,model.l[1]))
end

function visualize!(vis, model::RobotZoo.Acrobot, x::StaticVector)
    e1 = @SVector [1,0,0]
    q1,q2 = expm((x[1]-pi/2)*e1), expm(x[2]*e1)
    settransform!(vis["robot","link1"], LinearMap(UnitQuaternion(q1)))
    settransform!(vis["robot","link1","link2"], compose(Translation(0,0,model.l[1]), LinearMap(UnitQuaternion(q2))))
end

# Dubins Car
function _set_mesh!(vis, model::RobotZoo.DubinsCar; 
        color=nothing, color2=nothing, height=0.05, radius=model.radius)
    radius = Float32(radius)
    body = Cylinder(Point3f0(0,0,0), Point3f0(0,0,height), radius)
    face = Rect3D(Vec(3radius/4, -radius/2, 0), Vec(radius/4, radius, height*1.1))
    c1,c2 = defcolor(color, color2, colorant"blue", colorant"yellow")
    setobject!(vis["geom"]["body"], body, MeshPhongMaterial(color=c1))
    setobject!(vis["geom"]["face"], face, MeshPhongMaterial(color=c2))
end

# Quadrotor
function _set_mesh!(vis, model::RobotZoo.Quadrotor; scaling=1.0, color=colorant"black")
    urdf_folder = joinpath(@__DIR__, "..", "data", "meshes")
    # if scaling != 1.0
    #     quad_scaling = 0.085 * scaling
    obj = joinpath(urdf_folder, "quadrotor_scaled.obj")
    if scaling != 1.0
        error("Scaling not implemented after switching to MeshCat 0.12")
    end
    robot_obj = MeshFileGeometry(obj)
    mat = MeshPhongMaterial(color=color)
    setobject!(vis["geom"], robot_obj, mat)
end

# Yak Plane
function _set_mesh!(vis, ::RobotZoo.YakPlane; color=nothing)
    # meshfile = joinpath(@__DIR__,"..","data","meshes","piper","piper_pa18.obj")
    # meshfile = joinpath(@__DIR__,"..","data","meshes","cirrus","Cirrus.obj")
    meshfile = joinpath(@__DIR__,"..","data","meshes","piper","piper_scaled.obj")
    jpg = joinpath(@__DIR__,"..","data","meshes","piper","piper_diffuse.jpg")
    if isnothing(color)
        img = PngImage(jpg)
        texture = Texture(image=img)
        mat = MeshLambertMaterial(map=texture) 
    else
        mat = MeshPhongMaterial(color=color)
    end
    obj = MeshFileGeometry(meshfile)
    # obj.vertices .= obj.vertices .* scaling
    setobject!(vis["geom"], obj, mat)
    settransform!(vis["geom"], compose(Translation(0,0,0.07),LinearMap( RotY(pi/2)*RotZ(-pi/2) )))
end

# _set_mesh!(vis, model::TrajectoryOptimization.InfeasibleModel) = _set_mesh!(vis, model.model)
function _set_mesh!(vis, ::RobotZoo.Satellite; dims=[1,1,2]*0.5, color=colorant"grey70")
    obj = Rect3D(Vec((-dims/2)...), Vec(dims...))
    mat = MeshPhongMaterial(; color=color)
    setobject!(vis["geom"], obj, mat)
end

function _set_mesh!(vis, model::RigidBody; dims=[1,1,2]*0.5, color=colorant"grey70")
    obj = Rect3D(Vec((-dims/2)...), Vec(dims...))
    mat = MeshPhongMaterial(; color=color)
    setobject!(vis["geom"], obj, mat)
end


