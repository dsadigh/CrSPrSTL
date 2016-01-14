function test_vehicle_rrt
    rng(1);
    dt = 0.2;
    SIZE = 0.5;
    R = 4;
    W = 1;
    
    map = figure;
    hold on;
    axis([-R-W,R+W,-R-W,R+W]);
    axis equal;
    grid on;
    
    theta1 = 0.:0.01:2*pi;
    theta2 = 2*pi:-0.01:0;
    xs = [cos(theta1)*(R+W/2) cos(theta2)*(R-W/2)];
    ys = [sin(theta1)*(R+W/2) sin(theta2)*(R-W/2)];
    patch(xs, ys, [0. 0. 0.], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');

    % For drawing purposes
%     for i=1:N
%         x1 = cos(2*pi*(i-1)/N)*R;
%         y1 = sin(2*pi*(i-1)/N)*R;
%         x2 = cos(2*pi*i/N)*R;
%         y2 = sin(2*pi*i/N)*R;
%         env.add_road(Road([x1 y1], [x2 y2], W, 'both'));
%     end

    rot = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

    M = 10;
    
    obstacles = {};
    
    for i = 1:M
        theta = 2*pi*(i-1)/M;
        obstacles{end+1} = rot(pi+pi/10+theta)*[-W/2 0 W/2; R-W R R-W];
        obstacles{end+1} = rot(pi+pi/10+pi/10+theta)*[-W/2 0 W/2; R+W R R+W];
    end
    
    for i = 1:length(obstacles)
        patch(obstacles{i}(1, :), obstacles{i}(2, :), '', 'EdgeColor', 'none', 'FaceColor', [1. 0.5 0]);
    end
        
    function result = safe(state)
        x = state(1:2);
        result = true;
        if norm(x)>R+W/2-SIZE/2
            result = false;
        end
        if norm(x)<R-W/2+SIZE/2
            result = false;
        end
        for k = 1:length(obstacles)
            tsum = 0;
            for j = 1:size(obstacles{k}, 2)
                p = obstacles{k}(:, j) - x(1:2);
                p = p(1)+p(2)*1i;
                q = obstacles{k}(:, mod(j, size(obstacles{k}, 2))+1) - x(1:2);
                q = q(1)+q(2)*1i;
                tsum = tsum + atan2(imag(q/p), real(q/p));
            end
            if abs(tsum)<0.1
                continue
            end
            result = false;
        end
    end
    
    function result = next(x)
        result = [x(1)+x(4)*cos(x(3))*dt; x(2)+x(4)*sin(x(3))*dt; x(3); x(4)];
    end
%     for i = 1:M
%         theta = 2*pi*(i-1)/M;
%         env.add_obstacle(VehicleObstacle(rot(pi+pi/10+theta)*[-W/2 0 W/2; R-W R-W/5 R-W]));
%         env.add_obstacle(VehicleObstacle(rot(pi+pi/10+pi/10+theta)*[-W/2 0 W/2; R+W R+W/5 R+W]));
%     end

    %objective = @(x0, t0, dt) Sum(@(t) x(t, 1)*x0(2)-x(t, 2)*x0(1));
    %sys.set_dyn_objective(objective);

    % Remain inside roads
%     N = 20;
%     sys.add_constraint(always(P('norm(x(t, 1:2))<=R+W/2-SIZE/2'), 3*dt, inf));
%     p = [];
%     for i=1:N
%         theta = 2*pi*i/N;
%         q = P('cos(theta)*x(t, 1)+sin(theta)*x(t, 2)>=R-W/2+SIZE/2');
%         if i==1
%             p = q;
%         else
%             p = or(p, q);
%         end
%     end
    %sys.add_constraint(always(p, 3*dt, inf));

    % Avoid obstacles
%     for i=1:numel(env.obstacles)
%         points = env.obstacles(i).points;
%         cons = [];
%         for j=1:size(points, 2)
%             p = points(:, j)';
%             q = points(:, mod(j, size(points, 2))+1)';
%             v = [p(2)-q(2) q(1)-p(1)];
%             v = v / norm(v);
%             C = P('(x(t, 1)-p(1))*v(1)+(x(t, 2)-p(2))*v(2)<=-SIZE/3');
%             if j==1
%                 cons = C;
%             else
%                 cons = or(cons, C);
%             end
%         end
%         %sys.add_constraint(always(cons, 3*dt, inf));
%     end

    % Initial conditions
%     sys.add_constraint(P('x(0)==[0; -R; 0; 0.5]'));
%     sys.add_constraint(P('u(0)==0'));

    start = [rot(pi/30)*[0; -R]; 0; 1];
    finish = [0; R; pi; 1];
    
    start_arrow = quiver(0, 0, 0, 0, 0, 'LineWidth', 5);
    goal_arrow = quiver(0, 0, 0, 0, 0, 'LineWidth', 5);
    
    function update_arrow(arrow, x)
        set(arrow, 'XData', x(1));
        set(arrow, 'YData', x(2));
        set(arrow, 'UData', cos(x(3))*x(4)*0.5);
        set(arrow, 'VData', sin(x(3))*x(4)*0.5);
    end

    RRT = {};
    
    function n = node(x, uprev, iprev)
        n.x = x;
        n.uprev = uprev;
        n.iprev = iprev;
    end

    RRT{end+1} = node(start, [], -1);
    
    function result = dist(x1, x2)
        result = abs(x1(1)-x2(1))+abs(x1(2)-x2(2))+0.03*abs(x1(3)-x2(3))+0.03*abs(x1(4)-x2(4));
    end

%     function result = dist2(x1, x2)
%         %result = norm(x1(1:2)-x2(1:2))^2;
%         [t1, r1] = cart2pol(x1(1), x1(2));
%         [t2, r2] = cart2pol(x2(1), x2(2));
%         dtheta = t2-t1;
%         while dtheta<0
%             dtheta = dtheta + pi;
%         end
%         result = ((r1-r2))^2+(2*R*dtheta)^2;
%     end
    
    function istart = closest(x)
        istart = -1;
        for j=1:length(RRT)
            if ~safe(RRT{j}.x) || ~safe(next(RRT{j}.x)) || ~safe(next(next(RRT{j}.x)))
                continue
            end
            if ~safe((RRT{j}.x+x)/2.)
                continue
            end
            dtheta = -cart2pol(RRT{j}.x(1), RRT{j}.x(2))+cart2pol(x(1), x(2));
            while dtheta<0
                dtheta = dtheta + 2*pi;
            end
            if dtheta > pi
                continue
            end
            if istart == -1 || dist(RRT{j}.x, x)<dist(RRT{istart}.x, x)
                istart = j;
            end
        end
    end
    
    function route(goal, istart)
        if nargin<=1
            istart = closest(goal);
        end
        if istart == -1
            return
        end
        update_arrow(start_arrow, RRT{istart}.x);
        update_arrow(goal_arrow, goal);
        t1 = 0;
        t2 = 1;
        xstart = RRT{istart}.x;
        sys = Vehicle(dt, SIZE);
        [x, u, ~] = sys.signals();
        sys.add_constraint(always(P(@(t, dt) abs(u(t, 1))<=1)));
        sys.add_constraint(always(P(@(t, dt) abs(u(t, 2))<=2)));
        sys.add_constraint(always(P(@(t, dt) abs(x(t, 4))<=1)));
        sys.add_constraint(P(@(t, dt) x(t1)==xstart));
        sys.set_objective(Sum(@(t, dt) dist(x(t), goal)));
        %try
            iprev = istart;
            sys.initialize(t1);
            for t=t1:dt:t2
                sys.find_control(5, t);
                sys.advance(t);
                if ~safe(sys.history.x(:, end))
                    return
                end
                newx = sys.history.x(:, end);
                if norm(newx(1:2)-goal(1:2))<0.1
                    break
                end
                RRT{end+1} = node(newx, sys.history.u(:, end), iprev);
                plot([sys.history.x(1, end), RRT{iprev}.x(1)], [sys.history.x(2, end), RRT{iprev}.x(2)], 'm');
                iprev = length(RRT);
            end
        %catch
        %    return
        %end
    end

    function x = random_point()
        while true
            x = [(rand()*2.-1)*(R+W); (rand()*2.-1)*(R+W); 0; rand()*2.];
            x(3) = atan2(x(1), -x(2))+rand();
            while x(3)>pi
                x(3) = x(3)-pi;
            end
            while x(3)<-pi
                x(3) = x(3)+pi;
            end
            if safe(x)
                break;
            end
        end
    end

    

    %route(finish, 1);
    while true
        route(random_point());
    end
    %route(finish);
    
    %sys.add_constraint(P(@(t, dt) x(2)==final));

    %env.fig_dir = 'loop_with_obstacles';
    %env.run_closed_loop(20, 0, 0.3);
    %env.save_movie('loop_with_obstacles.avi', 2);

end