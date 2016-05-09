rng(1);
dt = 1;
env = StochasticEnvironment(dt);

M = 2; % Number of segments on the freeway
beta = [0.2; 0.]; % Fraction of vehicles leaving the exit
fmax = [60.; 60.]; % Maximum flow passing through
nmax = [400.; 400.]; % Maximum number of vehicle in each segment
v = [.7; .7]; % Parameter of the traffic curve
w = [.2; .2]; % Parameter of the traffic curve

n = env.signal('stochastic', M);
f0 = env.signal(makedist('Normal','mu',40,'sigma',0)); % This is a r.v. signal taking uniform values on [0, 1]
f = env.signal('stochastic', M);
u = env.signal(M); % u is the control and is a deterministic signal; it's the rate at which we allow on-ramp cars to proceed
r = env.signal('stochastic', M); % r is the actual rate at which cars proceed. r=u unless there are no more cars on the on-ramp
s = env.signal('stochastic', M);
l = env.signal('stochastic', M);
d = env.signal(makedist('Normal','mu',15,'sigma',0), M);
travel_time = env.signal('stochastic');


env.add_plot(n);
env.add_plot(f0, 'f(0)');
%env.add_plot(f);
%env.add_plot(l);
env.add_plot(u);
env.add_plot(d);
%env.add_plot(r);
%env.add_plot(travel_time, 'Travel Time');
env.ncols = 2; % Number of columns in the plot

env.add_constraint(always(P(@(t, dt) travel_time(t)==sum(n(t))+sum(l(t)))));

%c1 = P(@(t, dt) n(0)==0) & P(@(t, dt) f(0)==0) ...
%   & P(@(t, dt) s(0)==0) & P(@(t, dt) l(0)==0); % Initial condition
c1 = P(@(t, dt) n(0) == 50) & P(@(t, dt) l(0)==0);
b1 = always(P(@(t, dt) u(t)<=20.)); % Bound on the control

p1 = always(P(@(t, dt) n(t+dt) == n(t)+[f0(t); f(t, 1:M-1)]+r(t)-f(t)-s(t))); % Traffic dynamics
p2 = always(P(@(t, dt) l(t+dt) == l(t)+d(t)-r(t))); % On-ramp dynamics
p3 = always(P(@(t, dt) s(t) == beta.*(s(t)+f(t)))); % Off-ramp dynamics
p4 = always(P(@(t, dt) u(t) >= r(t))&P(@(t, dt) r(t) >= 0)); % p4, p5, and p6 ensure that whatever we allow on the on-ramp goes through
p5 = always(P(@(t, dt) l(t+dt) >= 0));
p6 = AndPredicate();
for i=1:M
    p6 = p6&always(P(@(t, dt) u(t, i)==r(t, i))|P(@(t, dt) l(t+dt, i)==0));
end
p7 = always(P(@(t, dt) f(t) <= fmax));
p8 = always(P(@(t, dt) f(t) <= (1-beta).*v.*n(t)));
p9 = AndPredicate();
p10 = AndPredicate();
for i=1:M
    if i<M
        p9 = p9 & always(P(@(t, dt) f(t, i) <= w(i+1)*(nmax(i+1)-n(t, i+1))));
    end
    a = P(@(t, dt) f(t, i)==fmax(i));
    b = P(@(t, dt) f(t, i)==(1-beta(i))*v(i)*n(t, i));
    if i<M
        c = P(@(t, dt) f(t, i)==w(i+1)*(nmax(i+1)-n(t, i+1)));
        p10 = p10&always(a|b|c);
    else
        p10 = p10&always(a|b);
    end
end

env.add_constraint(b1);
env.add_constraint(c1);
env.add_constraint(p1&p2&p3&p4&p5&p6&p7&p8&p9&p10);
% Example objective: Sum of waiting times
%env.set_objective(Expectation(Sum(@(t, dt) sum(l(t))+sum(n(t))+0.001*sum(u(t)))));
% Another example objective: Variance of waiting times
env.set_objective(Expectation(Variance(@(t, dt) sum(l(t))+sum(n(t)))));
env.plot_uncertainty = false;
env.run(5, 0, 20, 2); % First param = horizon length, second param = t1, third param = t2, fourth param = number of samples
past = env.past;
past.filled(u(1)) % Value of n at t=3
past.filled(u(2))