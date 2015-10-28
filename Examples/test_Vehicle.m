%% test_Vehicle.m Testing Reaching a point

close all
clear all
dt = 0.03;
T0 = 0.;
T1 = 5.; %5
L = 10; %20
% Defining a Vehicle system and satisfying contraints and objective
sys = Vehicle(dt);
[x u y] = sys.signals();
% Reaching a fixed point
sys.set_objective('norm(x(t, 1:2)-[1; 1])^2');
% Following a time-varying point
%sys.set_objective('norm(x(t, 1:3)-[t; 0; 0])^2+2*norm(x(t, 7:9))^2');
sys.add_constraint(P('x(0)==zeros(6,1)'));
sys.add_constraint(P('u(0)==zeros(2, 1)'));
sys.add_constraint(always(P('abs(u(t, 1:2))<=0.3')));
sys.run_closed_loop(L,T0,T1);
%sys.run_open_loop(T0, T1)




