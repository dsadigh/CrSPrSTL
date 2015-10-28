%% test_simple_cars.m: Testing two vehicles passing each other at an intersection.
close all
clear all

dt = 0.1; 

A1 = [0 1; 0 0];
B1 = [0; 1];

A2 = [0 1; 0 0];
B2 = [0; 1];

A = [A1 zeros(2); zeros(2) A2];

Bu = [B1 zeros(2,1); B2 zeros(2,1)];

C = [1 1 1 1];
Du = 0;

%%% Classfier %%%
RNG = 0:0.01:1; 
Xdim1 = repmat(RNG, length(RNG), 1);
Xdim2 = repmat(RNG', 1, length(RNG));
% defining true obstacle
Ytrue = (Xdim2 <= 0.3) - (Xdim2 > 0.3);
% training data for classifer
Xtraining = [0.2 0.3; 0.8 0.3; 0.2 0.7; 0.7 0.6];
Ytraining = zeros(size(Xtraining, 1), 1);
for i = 1:size(Xtraining, 1)
    Ind_fun = (Xdim1 == Xtraining(i, 1)).*(Xdim2 == Xtraining(i, 2));
    [xind, yind] = find(Ind_fun == 1);    
    Ytraining(i) = Ytrue(xind, yind);
end
Xtraining = [Xtraining ones(length(Ytraining), 1)];
[m, S] = linearGP(Xtraining, Ytraining, 1);
%%%%

n = normal(m, S);
%n = normal(0.5, 0.1^2);


sys = System(dt);
sys.set_lti_dynamics(A, Bu, C, Du);

[x, u, y] = sys.signals();

sys.add_constraint(P('x(0)==[-10; 0; 10; 0]'));
sys.add_constraint(always(P('abs(u(t))<=1')));
sys.add_constraint(always(implies(P('abs(x(t,1) - x(t,3))<2'),always(P('abs(x(t,2)) < 0.1'),0.,2.))))

%sys.add_constraint(Pr('x(t,1)*n > 0') >= 0.9)


sys.set_objective('1');
%sys.run_open_loop(0., 5.);
sys.run_closed_loop(30, 0., 10.);