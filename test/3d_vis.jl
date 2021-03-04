# Test basic visualization of a single pose
model = RobotZoo.Quadrotor()
vis = Visualizer()
if !haskey(ENV, "CI")
    open(vis)
    wait(vis)
end
TrajOptPlots.set_mesh!(vis, model)
x = RBState([1,1,1], expm(SA[1,0,0]*deg2rad(30)), zeros(3), zeros(3))
x0 = zero(RBState)
visualize!(vis, x)
visualize!(vis, x0)
visualize!(vis, model, SVector(x))

# Visualize a trajectory
delete!(vis)
prob = DubinsCar(:escape)
model = prob.model
TrajOptPlots.set_mesh!(vis, model)
N = prob.N 
times = TrajectoryOptimization.get_times(prob)
X = [@SVector [sin(t), cos(t), -t] for t in times]
U = [@SVector zeros(2) for k = 1:N-1]
Z = Traj(X, U, push!(diff(times),0))

visualize!(vis, model, Z)
visualize!(vis, model, times[end]/4, states(Z))

initial_states!(prob, X)
initial_controls!(prob, U)
visualize!(vis, prob)

traj3!(vis, prob)
traj3!(vis, prob, inds=1:3)
traj3!(vis, prob, inds=SA[1,2,3])
traj3!(vis, prob, inds=[1,2,3])
traj3!(vis, prob, linewidth=4)
traj3!(vis, prob, linewidth=4, color=colorant"red")



# Vis 2 at the same time
X2 = [@SVector [2sin(t), 2cos(t), -t] for t in times]
Z2 = Traj(X2, U, push!(diff(times),0))
visualize!(vis, model, times[end], X2)

visualize!(vis, model, Z, Z2)
visualize!(vis, model, times[end], X, X2)
prob2 = Problem(prob.model, prob.obj, prob.xf, prob.tf, X0=X2, U0=U)
visualize!(vis, prob, prob2)
delete!(vis["robot_copies"])

# Waypoints
@test_throws ArgumentError TrajOptPlots.waypoints!(vis, model, Z)
TrajOptPlots.waypoints!(vis, model, Z, length=5)
TrajOptPlots.waypoints!(vis, model, Z, length=10)
TrajOptPlots.waypoints!(vis, model, Z, inds=[1,51,75,101])
TrajOptPlots.waypoints!(vis, model, Z, length=11, color=colorant"red", color_end=colorant"green")
delete!(vis["waypoints"])
TrajOptPlots.clear_waypoints!(vis)

# Plot the cylinders
TrajOptPlots.add_cylinders!(vis, prob, height=1)
TrajOptPlots.add_cylinders!(vis, prob, height=0.25, color=colorant"forestgreen")

# Add a point
TrajOptPlots.add_point!(vis, [1,1,1], radius=1)
TrajOptPlots.add_point!(vis, [1,1,1], radius=1, color=colorant"purple")
TrajOptPlots.add_point!(vis, [3,1,1], radius=0.5, color=colorant"cyan", name="point2")