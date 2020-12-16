vis = Visualizer()
if !haskey(ENV, "CI")
    open(vis)
    wait(vis)
end

model = RobotZoo.DoubleIntegrator(2)
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[1,2,0,0.])
TrajOptPlots.set_mesh!(vis, model, color=colorant"pink")
TrajOptPlots.set_mesh!(vis, model, radius=0.5, height=0.5) 

delete!(vis)
model = RobotZoo.Pendulum()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[pi/2,0])
TrajOptPlots.set_mesh!(vis, model, color=colorant"green")
TrajOptPlots.set_mesh!(vis, model, color2=colorant"green")
TrajOptPlots.set_mesh!(vis, model, color=colorant"red", color2=colorant"cyan")

delete!(vis)
model = RobotZoo.Cartpole()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[-0.5, pi/2, 0,0])

model2 = RobotZoo.Cartpole(0.2,0.5,1.0,9.81)  # make it longer
TrajOptPlots.set_mesh!(vis, model2)
visualize!(vis, model, SA[-0.5, pi/2, 0,0])
TrajOptPlots.set_mesh!(vis, model2, color=colorant"purple")
TrajOptPlots.set_mesh!(vis, model2, color=colorant"purple", color2=colorant"yellow")

delete!(vis)
model = RobotZoo.Acrobot()
TrajOptPlots.set_mesh!(vis, model, color=colorant"purple")
visualize!(vis, model, SA[pi/4, pi/4, 0,0])

model = RobotZoo.Acrobot(l=SA[0.5,1.0])
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[pi/4, pi/4, 0,0])

delete!(vis)
model = RobotZoo.DubinsCar()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[0.5, 0, pi/4])

model = RobotZoo.DubinsCar(0.5)
TrajOptPlots.set_mesh!(vis, model, height=0.2)
TrajOptPlots.set_mesh!(vis, model, radius=0.1)
TrajOptPlots.set_mesh!(vis, model, color=colorant"cyan") 
TrajOptPlots.set_mesh!(vis, model, color=colorant"orange", color2=colorant"gray30") 

delete!(vis)
model = RobotZoo.Quadrotor{MRP}()
TrajOptPlots.set_mesh!(vis, model)
g = MRP(expm(SA[1,0,0]*pi/4))
visualize!(vis, model, SA[0,0,1, g.x, g.y, g.z, 0,0,0, 0,0,0])
TrajOptPlots.set_mesh!(vis, model, color=colorant"white")

delete!(vis)
model = RobotZoo.YakPlane{MRP}()
TrajOptPlots.set_mesh!(vis, model)
g = MRP(expm(SA[1,0,0]*(pi/4 + pi)))
visualize!(vis, model, SA[0,0,1, g.x, g.y, g.z, 0,0,0, 0,0,0])
TrajOptPlots.set_mesh!(vis, model, color=colorant"cyan")

import RobotZoo.Satellite
Base.position(::Satellite, x) = @SVector zeros(3)
RobotDynamics.orientation(::Satellite, x) = UnitQuaternion(x[4], x[5], x[6], x[7])

delete!(vis)
model = RobotZoo.Satellite()
TrajOptPlots.set_mesh!(vis, model)
q = rand(UnitQuaternion)
visualize!(vis, model, SA[0,0,0, Rotations.params(q)...])

delete!(vis)
TrajOptPlots.set_mesh!(vis, model, dims=[0.5,0.5,1]*2, color=colorant"red")

# New Rigid Body
struct MyRB{R} <: RobotDynamics.RigidBody{R} end
delete!(vis)
model = MyRB{MRP}()
TrajOptPlots.set_mesh!(vis, model, dims=[0.5,0.5,1]*4, color=colorant"darkblue")

model = RobotZoo.DoubleIntegrator(2)
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[1,2,0,0.])
