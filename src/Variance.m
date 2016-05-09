classdef Variance < Objective

    properties
        f
    end
    
    methods
        function self = Variance(f)
            self.f = extract_function(f);
        end
        function result = value(self, ts, dt, ~)
            sum1 = 0;
            sum2 = 0;
            for t=ts
                v = self.f(t, dt);
                sum1 = sum1 + v;
                sum2 = sum2 + v^2;
            end
            sum1 = sum1/numel(ts);
            sum2 = sum2/numel(ts);
            result = sum2-sum1^2;
        end
    end
    
end

