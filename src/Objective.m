classdef (Abstract) Objective
    % class: Objective
    % methods: minimize: optimizes a value function given a set of constraints using
    % gurobi as a solver.
    
    methods (Abstract)
        V = value(self, ts, dt)
    end
    methods
        function result = minimize(self, varargin)
            [dt, ts, p] = extract_args(varargin{:});
            C = p.enforce(dt, ts(1), ts(end), ts(1), ts(1));
            %[result reordering consExpansion] = optimizeFB(C, self.value(ts, dt), sdpsettings('solver', 'gurobi', 'usex0', 1));
            %result = optimize(C, self.value(ts, dt), sdpsettings('solver', 'gurobi', 'usex0', 1));
             result = optimize(C, self.value(ts, dt), sdpsettings('solver', 'gurobi'));
        end
    end
end

function [dt, ts, p] = extract_args(varargin)
    if isa(varargin{end}, 'Predicate')
        dt = extract_dt(varargin{1:end-1});
        ts = varargin{1};
        p = varargin{end};
    else
        dt = extract_dt(varargin{:});
        ts = varargin{1};
        p = AndPredicate();
    end
end