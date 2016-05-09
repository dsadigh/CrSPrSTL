classdef Vehicle < System
    % class: Vehicle: Subclass of system used for defining the nonlinear 
    %                   dynamics of a point-mass model of a vehicle
    % methods: plotter, Vehicle
    properties
        size
        sensor
    end
    
    methods
        function set_sensor(self, sensor, obstacle_model)
            self.sensor = sensor;
            function C = dyn_constraint(x, t)
                self.sensor.sense(obstacle_model, x, t);
                C = always(Pr(@(t, dt) self.sensor.wall'*[self.x(t, 1:2); 1]<=0)>=self.sensor.threshold, 2*self.dt, inf);
            end
            self.add_dyn_constraint(@dyn_constraint);
        end
        function self = Vehicle(dt, size)
            self@System(dt);
            self.size = size;
            dyn = Dynamics(4, 2);
            [x, u] = dyn.symbols();
            % State: x, y, heading, speed
            % Control: steer, acceleration
            f1 = x(4)*cos(x(3));
            f2 = x(4)*sin(x(3));
            f3 = x(4)*u(1)/self.size;
            f4 = u(2);
            dyn.set_f([f1; f2; f3; f4]);
            dyn.set_g(x);
            self.set_dynamics(dyn);
            self.sensor = [];
        end
    end
    
end

