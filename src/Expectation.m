classdef Expectation < Objective
    %EXPECTATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        obj
    end
    
    methods
        function self = Expectation(obj)
            self.obj = obj;
        end
        function result = value(self, ts, dt, samples)
            val = self.obj.value(ts, dt, samples);
            result = 0;
            for sample=samples
                result = result + sample.filled(val);
            end
            result = result/numel(samples);
        end
    end
    
end

