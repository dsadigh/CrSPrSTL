%% test_Quadrotor.m Testing Reference Tracking with quadratic cost
close all
clear all

dt = 0.03;
T0 = 0.;
T1 = 10.; %5
L = 10; %20

% Set a reference Trajectory to follow:
xref = zeros(3, 2*L+round((T1-T0)/dt));
uref = zeros(4, 2*L+round((T1-T0)/dt));

for i=1:size(xref,2)
    %xref(1,i) = i*dt;
    %xref(2,i) = i*dt;
    xref(1, i) = i*dt;
    xref(2, i) = sin(i*dt);
end

Q = eye(3);
R = zeros(4);

% Defining a quadrotor system and satisfying contraints and objective
sys = Quadrotor(dt, 0.1);
[x u y] = sys.signals();
% LQR Cost Function:
sys.set_objective(' transpose(x(t, 1:3) - xref(:,round(t/dt)+1)) * Q * (x(t, 1:3) - xref(:,round(t/dt)+1)) + 2*norm( x(t, 7:9) )^2 + transpose(u(t)) * R * u(t)');
sys.add_constraint(P('x(0)==zeros(12,1)'));
sys.add_constraint(P('u(0)==zeros(4, 1)'));
sys.add_constraint(always(P('abs(u(t, 1:2))<=0.3')));
sys.add_constraint(always(P('0<=u(t, 4)<=10')));
sys.run_closed_loop(L,T0,T1);
%sys.run_open_loop(T0, T1)

%% Testing reaching a point or following a time-varying point
dt = 0.03;
T0 = 0.;
T1 = 5.; %5
L = 10; %20
% Defining a quadrotor system and satisfying contraints and objective
sys = Quadrotor(dt, 0.1);
[x u y] = sys.signals();
% Reaching a fixed point
%sys.set_objective('norm(x(t, 1:3)-[1; 1; 0])^2+2*norm(x(t, 7:9))^2');
% Following a time-varying point
sys.set_objective('norm(x(t, 1:3)-[t; 0; 0])^2+2*norm(x(t, 7:9))^2');
sys.add_constraint(P('x(0)==zeros(12,1)'));
sys.add_constraint(P('u(0)==zeros(4, 1)'));
sys.add_constraint(always(P('abs(u(t, 1:2))<=0.3')));
sys.add_constraint(always(P('0<=u(t, 4)<=10')));
sys.run_closed_loop(L,T0,T1);
%sys.run_open_loop(T0, T1)
%sys.display_signal()

%% Testing reaching a point with a quadrotor with battery dynamics
dt = 0.03;
T0 = 0.;
T1 = 5.; %5
L = 10; %20
% Defining a quadrotor system and satisfying contraints and objective
sys = QuadrotorWithBattery(dt, 0.1);
[x u y] = sys.signals();
% Reaching a fixed point
sys.set_objective('norm(x(t, 1:3)-[0; 0; -1])^2+2*norm(x(t, 7:9))^2');
% Following a time-varying point
%sys.set_objective('norm(x(t, 1:3)-[t; 0; 0])^2+2*norm(x(t, 7:9))^2');
sys.add_constraint(P('x(0, 1:12)==zeros(12,1)'));
% Initialize battery as full:
sys.add_constraint(P('x(0, 13) == 10'));
sys.add_constraint(P('u(0)==zeros(4, 1)'));
sys.add_constraint(always(P('abs(u(t, 1:2))<=0.3')));
sys.add_constraint(always(P('0<=u(t, 4)<=10')));
sys.add_constraint(always(implies(P('x(t, 13) <= 5'), P('x(t, 3) >= -0.1'))));
value(x(0))
sys.run_closed_loop(L,T0,T1);
%sys.run_open_loop(T0, T1)

