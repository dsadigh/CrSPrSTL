dt = 0.1;
sys = Vehicle(dt, 0.5);
sys1 = Vehicle(dt, 0.5);
[x, u, y] = sys.signals();
env = VehicleEnvironment(sys);
%env.add_system(sys1);
env.add_road(Road([0.2 -2], [0.2 2], 0.4, 'both'));
env.add_road(Road([-0.2 2], [-0.2 -2], 0.4, 'both'));
env.add_road(Road([-2 -0.6], [2 -0.6], 0.4, 'right'));
env.add_road(Road([-2 -0.2], [2 -0.2], 0.4, 'left'));
env.add_road(Road([2 0.2], [-2 0.2], 0.4, 'left'));
env.add_road(Road([2 0.6], [-2 0.6], 0.4, 'right'));
env.add_road(Road([-1 0.6], [-3 2.6], 0.4, 'both'));
env.add_intersection(Intersection([-0.4 -0.8; 0.4 -0.8; 0.4 0.8; -0.4 0.8], @(t) t<=10));
%env.add_road(Road([0.2 0.5], [2.2, 2.5], 0.4, 'both'));
%env.add_road(Road([0 -1], -0.2, 0.4));
%env.add_road(Road([1 0], -0.2, 0.4));
%env.add_road(Road([-1 0], -0.2, 0.4));

sys.add_constraint(P('x(0)==[-0.2; 1.5; -pi/2; 0.5]'));
sys.add_constraint(P('u(0)==0'));
sys.add_constraint(always(P('abs(u(t, 1))<=2')));
sys.add_constraint(always(P('abs(u(t, 2))<=1')));
sys.add_constraint(always(P('abs(x(t, 4))<=1')));
sys.add_dyn_constraint(env.obey_lights(sys));
sys.add_dyn_constraint(env.remain_inside_roads(sys));
% p = {};
% for i=1:numel(env.roads)
%     pred = P(@(t, dt) env.roads(i).contains(x(t, 1:2), 0.1));
%     heading = env.roads(i).heading();
%     heading = heading - floor((heading-theta_0+pi)/(2*pi))*(2*pi);
%     pred = pred & P(@(t, dt) abs(x(t, 3)-heading)<=pi/2);
%     p{end+1} = pred; %#ok<SAGROW>
% end
% sys.add_constraint(always(or(p{:})));
sys.set_objective('norm(x(t, 1:2)-[-10; 0.2])+abs(u(t, 1))*0.01');

env.run_closed_loop(10, 0, 3);
env.save_movie('left_turn_no_cars.avi', 1);