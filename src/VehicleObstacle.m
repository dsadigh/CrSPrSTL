classdef VehicleObstacle
    %VEHICLEOBSTACLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points
    end
    
    methods
        function self = VehicleObstacle(points)
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

