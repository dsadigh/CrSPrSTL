classdef StochasticEnvironment < handle
    %STOCHASTICENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dt
        signals
        constraints
        objective
        base_sample
        plots
        ncols
        past
        plot_uncertainty
    end
    
    methods
        function self=StochasticEnvironment(dt)
            self.dt = dt;
            self.signals = {};
            self.constraints = {};
            self.objective = 0.;
            self.base_sample = Sample();
            self.plots = {};
            self.ncols = 1;
            self.plot_uncertainty = true;
        end
        function [varargout] = signal(self, varargin)
            for i=1:nargout
                s = Signal(self.dt, varargin{:});
                self.signals{end+1} = struct('signal', s, 'history', []);
                varargout{i} = s; %#ok<AGROW>
            end
        end
        function add_constraint(self, constraint)
            self.constraints{end+1} = constraint;
        end
        function set_objective(self, objective)
            self.objective = objective;
        end
        function set_value(x, val)
            self.base_sample.set_value(x, val);
        end
        function plotter = get_plotter(self)
            plotter = StochasticPlotter(self.plots, self.ncols);
            plotter.plot_uncertainty = self.plot_uncertainty;
        end
        function add_plot(self, signal, name)
            if nargin<=2
                name = inputname(2);
            end
            self.plots{end+1} = struct('signal', signal, 'name', name, 'length', length(signal.generator()));
        end
        function run(self, L, t1, t2, num_samples)
            for i=1:numel(self.signals)
                self.signals{i}.history = [];
            end
            past_sample = Sample(self.base_sample);
            samples = [];
            for i=1:num_samples
                samples = [samples Sample(past_sample)]; %#ok<AGROW>
            end
            plotter = self.get_plotter();
            plotter.reset();
            pause
            for t=t1:self.dt:t2
                for i=1:num_samples
                    samples(i).overwrite(past_sample);
                end
                T1 = max(t1, t-L*self.dt);
                T2 = t+L*self.dt;
                cur_sample = Sample(past_sample);
                self.objective.minimize(T1:self.dt:T2, AndPredicate(self.constraints{:}), samples, [samples cur_sample]);
                %for i=1:numel(self.signals)
                %    self.signals{i}.filled(
                %end
                for i=1:numel(self.signals)
                    past_sample.set_value(self.signals{i}.signal(t), value(cur_sample.filled(self.signals{i}.signal(t))));
                end
                plotter.capture_past(t, cur_sample);
                plotter.capture_future(t:self.dt:T2, samples);
            end
            self.past = past_sample;
        end
    end
end

