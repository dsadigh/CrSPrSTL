function test_dyn_constraints()
    % function: test_dyn_constraints(): testing the control of a system
    %                                   with dynamic constraints.
    % wall_constraint: Defines a dynamic constraint for the system. For
    %                  instance, the control only holds if the system is
    %                  close to it.
    
dt = 0.1;
sys = System(dt);


if true
    dyn = Dynamics(2, 1);
    [x, u] = dyn.symbols();
    dyn.set_f([x(2)+x(2)^3/100; u]);
    dyn.set_g(x(1));
    sys.set_dynamics(dyn);
else
    A = [0 1; 0 0];
    Bu = [0; 1];
    C = [1 0];
    D = 0;
    sys.set_lti_dynamics(A, Bu, C, D);
end

[x, u, y] = sys.signals();

function constraint = wall_constraint(x0)
    constraint = AndPredicate();
    if x0(1)>4
        constraint = always(P(@(t) x(t, 1)<=9));
    end
end

sys.add_dyn_constraint(@wall_constraint);

sys.add_constraint(P(@(t) x(0)==[0;0]));
sys.add_constraint(P(@(t) u(0)==0));
sys.add_constraint(always(P(@(t) abs(u(t))<=5)));
sys.set_objective(Sum(@(t) abs(x(t)-10)+abs(u(t))));

sys.run_closed_loop(30, 0., 5.);

end