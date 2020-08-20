
# Test circle constraint plotting
xc = [1,2,3.]
yc = [-1,-3,-2.]
rc = [0.5, 0.25, 0.75]
con = CircleConstraint(3, xc, yc, rc)
length(con)

con2 = GoalConstraint(@SVector rand(3))

plot(con, circle_color=:green)
plot(con2)

prob, = Problems.DubinsCar()
plot(prob.constraints)
add_constraint!(prob.constraints, con, 1:prob.N-1)
plot(prob.constraints)

# Make sure you can add it an existing plot
t = range(0,3,length=21)
p = plot(t, (t./2).^2)
plot!(prob.constraints)
@test length(p.series_list) == 12

# Dubins Car 
prob, = Problems.DubinsCar()
U0 = [SA[1.0, k/100] for k = 1:prob.N-1]
initial_controls!(prob, U0)
rollout!(prob)
plot(prob.model, prob.Z)
plot!(prob.constraints)
plot(prob)

# Double Integrator
prob,opts = Problems.DoubleIntegrator()
ilqr = iLQRSolver(prob, opts)
solve!(ilqr)
plot(prob.model, prob.Z)
plot(prob)