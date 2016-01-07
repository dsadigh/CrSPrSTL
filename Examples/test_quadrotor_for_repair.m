close all
clear all
% Testing reaching a point or following a time-varying point
dt = 0.03;
T0 = 0.;
T1 = 5.; %5
L = 10; %20
% Defining a quadrotor system and satisfying contraints and objective
sys = Quadrotor(dt, 0.1);
[x u y] = sys.signals();
% Reaching a fixed point
sys.set_objective('norm(x(t, 1:3)-[1; 1; -0.1])^2+2*norm(x(t, 7:9))^2');
% Following a time-varying point
%sys.set_objective('norm(x(t, 1:3)-[t; 0; 0])^2+2*norm(x(t, 7:9))^2');
sys.add_constraint(always(P('0>x(t,3)>-1.1')));% 
sys.add_constraint(eventually(P('x(t,2)>3')));
sys.add_constraint(P('x(0)==[0;0;-0.1;0;0;0;0;0;0;0;0;0]'));%
sys.add_constraint(P('u(0)==zeros(4, 1)'));%
sys.add_constraint(always(P('abs(u(t, 1:2))<=0.3')));
sys.add_constraint(always(P('0<=u(t, 4)<=5')));%
sys.run_closed_loop(L,T0,T1);
%sys.run_open_loop(T0, T1)
%sys.display_signal()



