%% test_predicates.m: Defining probabilistic and deterministic predicates and checkin satisfaction of a cost function.

dt = 0.2;
x = Signal(dt, 2);
u = Signal(dt, 2);
%P(@(t, dt) x(t+dt)==x(t)+dt*u(t))
dynamics = always(P('x(t+dt)==x(t)+dt*u(t)'))&always(P('norm(u(t))<=1'));
boundary = P('x(0)==[-1;0]')&P('x(10)==[1;0]');
% Deterministic obstacle
obstacle = always(P('x(t, 1)+x(t, 2)<-0.5')|P('x(t, 1)-x(t, 2)>0.5'));
% Probabilistic obstacle
n = normal(0.5, 0.1^2);
pobstacle = always(Pr('x(t, 1)+x(t, 2)<-n')>=.99 | Pr(@(t) x(t, 1)-x(t, 2)>n)>=0.99);
% Complicated obstacle
m1 = normal([1;1], eye(2)*0.1);
m2 = normal([1;-1], eye(2)*0.1);
cobstacle = always(Pr(@(t) m1'*x(t)<-0.5)>=0.90 | Pr(@(t) m2'*x(t)>0.5)>=0.9);
s = Sum('norm(u(t))^2');
s.minimize(0:dt:10, dynamics&boundary&cobstacle);

%% TODO: For testing adversarial constraints
% dt = 0.1;
% x = Signal(dt);
% w = Signal(dt);
% p = always(P('abs(w(t))<=0.1')&P('x(t+dt)==x(t)+w(t)'))&always(Adversary('w(t)'))&always(Adversary('x(t)'));
% %q = always(P('x(t+dt)==x(t)+w(t)'));
% Sum('x(t)').minimize(0:dt:1, q, p);