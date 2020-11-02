vis = Visualizer()
if !haskey(ENV, "CI")
    open(vis)
    wait(vis)
end

model = RobotZoo.DoubleIntegrator(2)
TrajOptPlots.set_mesh!(vis, model)
@which visualize!(vis, model, SA[1,2,0,0.])

delete!(vis)
model = RobotZoo.Pendulum()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[pi/2,0])

delete!(vis)
model = RobotZoo.Cartpole()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[-0.5, pi/2, 0,0])

delete!(vis)
model = RobotZoo.Acrobot()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[pi/4, pi/4, 0,0])

delete!(vis)
model = RobotZoo.DubinsCar()
TrajOptPlots.set_mesh!(vis, model)
visualize!(vis, model, SA[0.5, 0, pi/4])

delete!(vis)
model = RobotZoo.Quadrotor{MRP}()
TrajOptPlots.set_mesh!(vis, model)
g = MRP(expm(SA[1,0,0]*pi/4))
visualize!(vis, model, SA[0,0,1, g.x, g.y, g.z, 0,0,0, 0,0,0])

delete!(vis)
model = RobotZoo.YakPlane{MRP}()
TrajOptPlots.set_mesh!(vis, model)
g = MRP(expm(SA[1,0,0]*(pi/4 + pi)))
visualize!(vis, model, SA[0,0,1, g.x, g.y, g.z, 0,0,0, 0,0,0])

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
