function test_dyn_quadrotor()
    % function: test_dyn_quadrotor(): testing the control of a quadrotor
    %           with dynamic constraints and probabilistic constraints.

    
dt = 0.03;
sys = Quadrotor(dt, 0.1);
env = QuadrotorEnvironment(sys);
env.add_obstacle(Obstacle(@(x, t) x(3)<-0.35));
env.fig_dir = 'dyn_quadrotor';
%env.add_obstacle(Obstacle(@(x, t) (x(1)-0.5)^2+(x(2)-0.5)^2+3*(x(3)+0.5)^2<0.5));
assignin('base', 'sys', sys);
[x, u, ~] = sys.signals();

% Define sensors and set constraints for sensors:
[X, Y, Z] = meshgrid(-0.4:0.1:0.4, -0.4:0.1:0.4, -0.4:0.1:0.4);
sensor = Sensor([X(:) Y(:) Z(:)], 0.95);
sys.set_sensor(sensor, env.obstacle_model());

% Objective: Reach a point
sys.set_objective(@(t, dt) norm(x(t, 1:3)-[1; 1; 0])^2 + 2*norm(x(t, 7:9))^2);

% Initial States
sys.add_constraint(P(@(t, dt) x(0)==zeros(12,1)));
sys.add_constraint(P(@(t, dt) u(0)==zeros(4, 1)));

% Bounds on control
sys.add_constraint(always(P(@(t, dt) abs(u(t, 1:2))<=0.3)));
sys.add_constraint(always(P(@(t, dt) -10<=u(t, 4)<=10)));

env.run_closed_loop(20, 0., 1.);

%sys.history.x;
%sys.run_open_loop(0., 5)

env.save_movie('dyn_quadrotor/dyn_quadrotor.avi', 1);
% Video
%video = VideoWriter('test.avi', 'Uncompressed AVI');
%video.FrameRate = 1;
%open(video);
%writeVideo(video, env.movie);
%close(video);

end