dt = 0.1;
sys1 = System(dt);
sys1.set_lti_dynamics([0 1; 0 0], [0; 1]);
[x1, u1, y1] = sys1.signals();
sys1.add_constraint(P('x1(0)==0'));
sys1.add_constraint(always(P('abs(u1(t))<=1')));
sys1.set_objective('abs(x1(t)-1)');

sys2 = System(dt);
sys2.set_lti_dynamics([0 1; 0 0], [0; 1]);
[x2, u2, y2] = sys2.signals();
sys2.add_constraint(P('x2(0)==0'));
sys2.add_constraint(always(P('abs(u2(t))<=1')));
sys2.set_objective('abs(x2(t)+1)');

env = Environment();
env.add_system(sys1);
env.add_system(sys2);
env.run_closed_loop(20, 0, 5);