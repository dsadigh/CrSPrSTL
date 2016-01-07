function test_quadrotor_with_battery()
    % function: test_dyn_quadrotor(): testing the control of a quadrotor
    %           with dynamic constraints and probabilistic constraints.

    
dt = 0.03;
sys = QuadrotorWithBattery(dt, 0.1);
env = QuadrotorEnvironment(sys);
env.fig_dir = 'quadrotor_with_battery'
assignin('base', 'sys', sys);
[x, u, ~] = sys.signals();

% Objective: Reach a point
sys.set_objective(@(t, dt) norm(x(t, 1:3)-[1; 1; -0.9])^2 + 2*norm(x(t, 7:9))^2);

% Initial States
sys.add_constraint(P(@(t, dt) x(0)==[zeros(12,1); 10]));
sys.add_constraint(P(@(t, dt) u(0)==zeros(4, 1)));

% Bounds on control
sys.add_constraint(always(P(@(t, dt) abs(u(t, 1:2))<=0.3)));
sys.add_constraint(always(P(@(t, dt) 0<=u(t, 4)<=10)));

    n = standard_normal();
    r = 10.;

    function C = be_safe(~, t0)
        going_high = eventually(P(@(t, dt) x(t, 3)<=-0.1), 0, 0.3);
        have_battery = always(Pr(@(t, dt) (x(t, 13)+r*sqrt(t-t0)*n>=1))>=0.8);
        C = always(implies(going_high, have_battery));
    end

sys.add_dyn_constraint(@be_safe);
%sys.add_constraint(always(implies(eventually(P(@(t, dt) x(t, 3)<=-0.2), 0, 0.3), always(P(@(t, dt) x(t, 13)>=1), 0, 0.5))));
%sys.add_constraint(implies(eventually(P(@(t, dt) x(t, 3)<=-0.2), 0, 0.3), always(P(@(t, dt) x(t, 13)>=1.), 0, 1)));

env.run_closed_loop(20, 0., 1.);

%sys.history.x;
%sys.run_open_loop(0., 5)
env.save_movie('quadrotor_with_battery.avi', 1);

end