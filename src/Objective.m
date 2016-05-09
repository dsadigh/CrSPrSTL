classdef (Abstract) Objective
    % class: Objective
    % methods: minimize: optimizes a value function given a set of constraints using
    % gurobi as a solver.
    
    methods (Abstract)
        V = value(self, ts, dt, samples)
    end
    methods
        function result = minimize(self, varargin)
            [dt, ts, p, samples_obj, samples_c] = extract_args(varargin{:});
            if isempty(samples_c)
                C = p.enforce(dt, ts(1), ts(end), ts(1), ts(1));
            else
                C = [];
                for sample=samples_c
                    C = [C p.enforce(dt, ts(1), ts(end), ts(1), ts(1), sample)]; %#ok<AGROW>
                end
            end
            obj = self.value(ts, dt, samples_obj);
            %[result reordering consExpansion] = optimizeFB(C, self.value(ts, dt), sdpsettings('solver', 'gurobi', 'usex0', 1));
            %result = optimize(C, self.value(ts, dt), sdpsettings('solver', 'gurobi', 'usex0', 1));
            result = optimize(C, obj, sdpsettings('solver', 'gurobi', 'warning', 0, 'usex0', 1));
            %global total_time
            %total_time = result.solvertime + total_time %#ok<NOPRT>
        end
    end
end

function [dt, ts, p, samples_obj, samples_c] = extract_args(varargin)
    if isa(varargin{end}, 'Sample')
        samples_obj = varargin{end-1};
        samples_c = varargin{end};
        [dt, ts, p] = extract_reg_args(varargin{1:end-2});
    else
        samples_obj = [];
        samples_c = [];
        [dt, ts, p] = extract_reg_args(varargin{:});
    end
end

function [dt, ts, p] = extract_reg_args(varargin)
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