%% test_nonlinear.m: Testing nonlinear dynamics without linearizing.

% x is 2 dimensional and u is 1 dimensional:
dyn = Dynamics(2, 1);
[x, u] = dyn.symbols();
% symbolically define the dynamics:
dyn.set_f([x(2); u+u^3/100]);
dyn.set_g(x(1));

% Use the symbolically defined nonlinear dynamics:
sys = System(0.1);
sys.set_dynamics(dyn);
[x, u, y] = sys.signals();
sys.set_objective('abs(u(t))');
sys.add_constraint(always(P('abs(u(t+dt)-u(t))<=1')));
sys.add_constraint(P('x(0)==[0; 0]'));
sys.add_constraint(always(eventually(P('x(t, 1)<=-0.5'), 0., 2.)));
sys.add_constraint(always(eventually(P('x(t, 1)>=0.5'), 0., 2.)));

%sys.add_constraint(P('abs(u(t))<=5'));
sys.run_closed_loop(30, 0., 5.);