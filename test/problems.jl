
function DubinsCar(scenario=:three_obstacles; N=101)
    if scenario == :three_obstacles

        #  Car w/ obstacles
        model = RobotZoo.DubinsCar()
        n,m = size(model)

        N = 101 # number of knot points
        tf = 5.0
        dt = tf/(N-1)

        x0 = @SVector [0., 0., 0.]
        xf = @SVector [3., 3., 0.]

        Q = Diagonal(@SVector [1., 1., 1.])
        R = Diagonal(@SVector [0.5, 0.5])
        Qf = 10.0*Diagonal(@SVector ones(n))
        obj = LQRObjective(Q,R,Qf,xf,N)

        # create obstacle constraints
        r_circle_3obs = 0.25

        circle_x = 3*@SVector [0.25, 0.5, 0.75]
        circle_y = 3*@SVector [0.25, 0.5, 0.75]
        circle_r = @SVector fill(r_circle_3obs+model.radius, 3)

        obs = CircleConstraint(n, circle_x, circle_y, circle_r)
        bnd = BoundConstraint(n,m, u_min=[0,-3],u_max=[3,3])
        goal = GoalConstraint(xf)

        conSet = ConstraintList(n,m,N)
        add_constraint!(conSet, obs, 2:N-1)
        add_constraint!(conSet, bnd, 1:N-1)
        add_constraint!(conSet, goal, N:N)

        # Create problem
        U = [@SVector fill(0.01,m) for k = 1:N-1]
        car_3obs_static = Problem(model, obj, xf, tf, constraints=conSet, x0=x0)
        initial_controls!(car_3obs_static, U)
        rollout!(car_3obs_static)
        return car_3obs_static

    elseif scenario==:escape

        # Static Car Escape
        T = Float64;

        # model
        model = RobotZoo.DubinsCar()
        n,m = size(model)
        x0 = @SVector [2.5,2.5,0.]
        xf = @SVector [7.5,2.5,0.]
        N = 101
        tf = 3.0

        # cost
        Q = (1e-3)*Diagonal(@SVector ones(n))
        R = (1e-2)*Diagonal(@SVector ones(m))
        Qf = 100.0*Diagonal(@SVector ones(n))
        obj = LQRObjective(Q,R,Qf,xf,N)

        # constraints
        r = 0.5
        s1 = 30; s2 = 50; s3 = 15

        circles_escape = NTuple{3,Float64}[]

        for i in range(0,stop=5,length=s1)
            push!(circles_escape,(0.,i,r))
        end
        for i in range(0,stop=5,length=s1)
            push!(circles_escape,(5.,i,r))
        end
        for i in range(0,stop=5,length=s1)
            push!(circles_escape,(10.,i,r))
        end
        for i in range(0,stop=10,length=s2)
            push!(circles_escape,(i,0.,r))
        end
        for i in range(0,stop=3,length=s3)
            push!(circles_escape,(i,5.,r))
        end
        for i in range(5,stop=8,length=s3)
            push!(circles_escape,(i,5.,r))
        end

        n_circles_escape = 3*s1 + s2 + 2*s3

        circles_escape
        x,y,r = collect(zip(circles_escape...))
        x = SVector{n_circles_escape}(x)
        y = SVector{n_circles_escape}(y)
        r = SVector{n_circles_escape}(r)

        obs = CircleConstraint(n,x,y,r)
        bnd = BoundConstraint(n,m,u_min=-5.,u_max=5.)
        goal = GoalConstraint(xf)

        conSet = ConstraintList(n,m,N)
        add_constraint!(conSet, obs, 2:N-1)
        add_constraint!(conSet, bnd, 1:N-1)
        add_constraint!(conSet, goal, N:N)

        # Build problem
        U0 = [@SVector ones(m) for k = 1:N-1]

        car_escape_static = Problem(model, obj, xf, tf;
            constraints=conSet, x0=x0)
        initial_controls!(car_escape_static, U0);

        X_guess = [2.5 2.5 0.;
                   4. 5. .785;
                   5. 6.25 0.;
                   7.5 6.25 -.261;
                   9 5. -1.57;
                   7.5 2.5 0.]
        # X0_escape = Altro.interp_rows(N,tf,Array(X_guess'))
        # initial_states!(car_escape_static, X0_escape)

        return car_escape_static
    end
end

function DoubleIntegrator()

    model = RobotZoo.DoubleIntegrator()
    n,m = size(model)

    # Task
    x0 = @SVector [0., 0.]
    xf = @SVector [1., 0]
    tf = 2.0

    # Discretization info
    N = 21
    dt = tf/(N-1)

    # Costs
    Q = 1.0*Diagonal(@SVector ones(n))
    Qf = 1.0*Diagonal(@SVector ones(n))
    R = 1.0e-1*Diagonal(@SVector ones(m))
    obj = LQRObjective(Q,R,Qf,xf,N)

    # Constraints
    u_bnd = 3.0
    x_bnd = [Inf,0.6]
    conSet = ConstraintList(n,m,N)
    bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd, x_min=-x_bnd, x_max=x_bnd)
    goal = GoalConstraint(xf)
    add_constraint!(conSet, bnd, 1:N-1)
    add_constraint!(conSet, goal, N:N)

    doubleintegrator_static = Problem(model, obj, xf, tf, constraints=conSet, x0=x0, N=N)
    TO.rollout!(doubleintegrator_static)
    return doubleintegrator_static
end
