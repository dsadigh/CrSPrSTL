dt = 1;
env = StochasticEnvironment(dt);

l = env.signal('stochastic', 2);
r = env.signal('stochastic', 2);
d = env.signal(makedist('Uniform', 'upper', 2), 2);

env.add_plot(l, 'l');
env.add_plot(r, 'r');
env.add_plot(d, 'd');

p = always(P(@(t, dt) l(t+dt) == l(t)+d(t))&P(@(t, dt) r(t)==0));

env.plot_uncertainty = true;
env.add_constraint(p);
env.set_objective(Expectation(Sum(@(t, dt) l(t))));
env.run(5, 0, 5, 3);
past = env.past;