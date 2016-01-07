dt = 0.1;
sys = Vehicle(dt, 0.5);
sys1 = Vehicle(dt, 0.5);

[x, u, y] = sys.signals();
env = VehicleEnvironment(sys);
env.add_system(sys1);

env.add_road(Road([0.2 -2], [0.2 -0.2], 0.4, 'both'));
env.add_road(Road([-0.2 -0.2], [-0.2 -2], 0.4, 'both'));
env.add_road(Road([-2 -0.2], [2 -0.2], 0.4, 'both'));
env.add_road(Road([2 0.2], [-2 0.2], 0.4, 'both'));
env.add_intersection(Intersection([-0.4 -0.4; 0.4 -0.4; 0.4 0.4; -0.4 0.4], @(t) t>=100));


theta_0 = -pi/2;
sys.add_constraint(P('x(0)==[0.2; -.9; pi/2; 0.5]'));
sys.add_constraint(P('u(0)==0'));
sys.add_constraint(always(P('abs(u(t, 1))<=2')));
sys.add_constraint(always(P('abs(u(t, 2))<=2')));
sys.add_constraint(always(P('abs(x(t, 4))<=1')));
sys.add_dyn_constraint(env.remain_inside_roads(sys));
env.fig_dir = 'right_turn'
%sys.add_dyn_constraint(env.avoid_crash_probabilistic(sys, 0.4, 0.8));
sys.add_dyn_constraint(env.avoid_crash(sys));

sys.set_objective('abs(x(t, 1)-10)+abs(x(t, 2)+0.2)+abs(x(t, 3)-pi/2)*0.05+abs(u(t, 1))*0.01');

[x, u, y] = sys1.signals();
sys1.add_constraint(P('x(0)==[-0.9; -0.2; 0; 0.8]'));
sys1.add_constraint(always(P('u(t)==[0; 0]'))); % If set to zero, feasible

env.run_closed_loop(20, 0, 4);
env.save_movie('right_turn.avi', 1);
%env.save_movie('right_turn_deterministic.avi', 1);