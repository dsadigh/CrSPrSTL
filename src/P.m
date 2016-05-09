classdef P < Predicate
    % class P
    % properties: f : composed predicate 
    % methods: Tconstraints, Fconstraints: Decomposes the predicate
    %           function and produces a set of yalmip constraints

    properties
        f
    end
    
    methods
        function self = P(f)
            self.f = extract_function(f);
        end
        
        function result = filled(self, C, varargin)
            if numel(varargin)>=1
                result = varargin{1}.filled(C);
            else
                result = C;
            end
        end
        
        function C = enforce(self, dt, l0, l1, t0, t1, varargin)
            l1 = l0+round((l1-l0)/dt)*dt;
            t0 = max(t0, l0);
            t1 = min(t1, l1);
            C = [];
            for i=0:round((t1-t0)/dt)
                t = t0+i*dt;
                C = [C, self.filled(self.f(t, dt), varargin{:})];  %#ok<AGROW>
            end
        end
        
        function C = Tconstraints(self, T, dt, t0, varargin)
            C = [];
            for i=1:numel(T)
                t = t0+(i-1)*dt;
                C = [C, implies(T(i), self.filled(self.f(t, dt), varargin{:}))]; %#ok<AGROW>
            end
        end
    
        function C = Fconstraints(self, F, dt, t0, varargin)
            C = [];
            for i=1:numel(F)
                t = t0+(i-1)*dt;
                C = [C, implies(self.filled(self.f(t, dt), varargin{:}), false(F(i)))]; %#ok<AGROW>
            end
        end
    end
end

