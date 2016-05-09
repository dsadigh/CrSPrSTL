classdef VehicleObstacle
    %VEHICLEOBSTACLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points
        color
    end
    
    methods
        function result = f(self, x, ~)
            result = true;
            for j=1:size(self.points, 2)
                p = self.points(:, j)';
                q = self.points(:, mod(j, size(self.points, 2))+1)';
                v = [p(2)-q(2) q(1)-p(1)];
                v = v / norm(v);
                if (x(1)-p(1))*v(1)+(x(2)-p(2))*v(2)>=0
                    result = false;
                end
            end
        end
        function self = VehicleObstacle(points, color)
            if nargin<=2
                color = [1. 0.5 0.];
            end
            self.color = color;
            s = 0;
            for i=2:size(points, 2)
                s = s + points(1, i-1)*points(2, i)-points(2, i-1)*points(1, i);
            end
            if s<0
                self.points = fliplr(points);
            else
                self.points = points;
            end
        end
    end
    
end

