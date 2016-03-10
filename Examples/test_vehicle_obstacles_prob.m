dt = 0.1;
SIZE = 0.5;
sys = Vehicle(dt, SIZE);
[x, u, y] = sys.signals();
env = VehicleEnvironment(sys);

N = 200;
R = 4;
W = 1;

% For drawing purposes
for i=1:N
    x1 = cos(2*pi*(i-1)/N)*R;
    y1 = sin(2*pi*(i-1)/N)*R;
    x2 = cos(2*pi*i/N)*R;
    y2 = sin(2*pi*i/N)*R;
    env.add_road(Road([x1 y1], [x2 y2], W, 'both'));
end

rot = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

PROBS = 1:2:20;
STATS = 2:2:20;

M = 10;
for i = 1:M
    theta = 2*pi*(i-1)/M;
    env.add_obstacle(VehicleObstacle(rot(pi+pi/10+theta)*[-W/2 0 W/2; R-W R-W/5 R-W]));
    env.add_obstacle(VehicleObstacle(rot(pi+pi/10+pi/10+theta)*[-W/2 0 W/2; R+W R+W/5 R+W]));
end

for i = 1:numel(env.obstacles)
    color = [1. 0.5 0.];
    if find(PROBS==i)
        color = [1. 0.5 0.7];
    end
    env.obstacles(i).color = color;
end

objective = @(x0, t0, dt) Sum(@(t) x(t, 1)*x0(2)-x(t, 2)*x0(1));
sys.set_dyn_objective(objective);

% Remain inside roads
N = 20;
sys.add_constraint(always(P('norm(x(t, 1:2))<=R+W/2-SIZE/2'), 3*dt, inf));
p = [];
for i=1:N
    theta = 2*pi*i/N;
    q = P('cos(theta)*x(t, 1)+sin(theta)*x(t, 2)>=R-W/2+SIZE/2');
    if i==1
        p = q;
    else
        p = or(p, q);
    end
end
sys.add_constraint(always(p, 3*dt, inf));

[X, Y] = meshgrid(-0.4:0.1:0.4, -0.4:0.1:0.4);
sensor = Sensor2D([X(:) Y(:)], 0.5);
sys.set_sensor(sensor, env.obstacle_model(env.obstacles(PROBS)));

% Avoid obstacles
for i=STATS
    points = env.obstacles(i).points;
    cons = [];
    for j=1:size(points, 2)
        p = points(:, j)';
        q = points(:, mod(j, size(points, 2))+1)';
        v = [p(2)-q(2) q(1)-p(1)];
        v = v / norm(v);
        C = P('(x(t, 1)-p(1))*v(1)+(x(t, 2)-p(2))*v(2)<=0');
        if j==1
            cons = C;
        else
            cons = or(cons, C);
        end
    end
    sys.add_constraint(always(cons, 3*dt, inf));
end

% Initial conditions
sys.add_constraint(P('x(0)==[0; -R; 0; 0.5]'));
sys.add_constraint(P('u(0)==0'));

% Bounds
sys.add_constraint(always(P('abs(u(t, 1))<=1')));
sys.add_constraint(always(P('abs(u(t, 2))<=2')));
sys.add_constraint(always(P('abs(x(t, 4))<=1')));

env.fig_dir = 'loop_with_obstacles';
env.run_closed_loop(20, 0, 10);
env.save_movie('loop_with_obstacles.avi', 2);
