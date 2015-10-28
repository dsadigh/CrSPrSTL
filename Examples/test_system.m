%% test_system.m: Defining probabilistic and deterministic predicates and preforming a Model Predictive Control using closed loop or open loop setting.
close all
clear all

dt = 0.1;
A = [0 1; 0 0];
Bu = [0; 1];
C = [1 0];
Du = 0;

n = normal(0.5, 0.1^2);

sys = System(dt);
sys.set_lti_dynamics(A, Bu, C, Du);

[x, u, y] = sys.signals();

sys.add_constraint(P('x(0)==[0; 0]'));
sys.add_constraint(always(P('abs(u(t))<=5')));
%sys.add_constraint(Pr('x(t,1)*n > 0') >= 0.9)
sys.add_constraint(always(eventually(P('x(t, 1)<=-0.5'), 0., 2.)));
sys.add_constraint(always(eventually(P('x(t, 1)>=0.5'), 0., 2.)));

sys.set_objective('abs(u(t))');
%sys.run_open_loop(0., 2.);
sys.run_closed_loop(30, 0., 10.);