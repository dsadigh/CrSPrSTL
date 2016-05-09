classdef StochasticPlotter < handle
    %STOCHASTICPLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fig
        plots
        ts
        ncols
        plot_uncertainty
    end
    
    methods
        function self = StochasticPlotter(plots, ncols, plot_uncertainty)
            if nargin<=1
                ncols = 1;
            end
            if nargin<=2
                plot_uncertainty = true;
            end
            self.plots = plots;
            self.fig = [];
            self.ncols = ncols;
            self.plot_uncertainty = plot_uncertainty;
        end
        function capture_past(self, t, sample)
            self.ts = [self.ts t];
            for i=1:numel(self.plots)
                self.plots{i}.values = [self.plots{i}.values value(sample.filled(self.plots{i}.signal(t)))];
                for j=1:self.plots{i}.length
                    set(self.plots{i}.past_plot(j), 'Xdata', self.ts, 'Ydata', self.plots{i}.values(j, :));
                end
            end
        end
        function capture_future(self, ts, samples)
            for i=1:numel(self.plots)
                
                for j=1:self.plots{i}.length
                    x = self.plots{i}.signal(ts, j);
                    if strcmp(self.plots{i}.signal.type, 'deterministic')
                        values = value(x);
                        set(self.plots{i}.mfuture_plot(j), 'Xdata', ts, 'Ydata', values);
                    else
                        values = [];
                        for k=1:numel(samples)
                            values = [values; value(samples(k).filled(x))]; %#ok<AGROW>
                        end
                        m = mean(values);
                        s = std(values);
                        set(self.plots{i}.mfuture_plot(j), 'Xdata', ts, 'Ydata', m);
                        if self.plot_uncertainty
                            set(self.plots{i}.sfuture_plot(j), 'Xdata', [ts fliplr(ts)], 'Ydata', [m-s fliplr(m+s)]);
                        end
                    end
                end
            end
        end
        function reset(self)
            self.ts = [];
            self.fig = figure;
            num_plots = 0;
            for i=1:numel(self.plots)
                num_plots = num_plots + self.plots{i}.length;
            end
            cur_plot = 1;
            for i=1:numel(self.plots)
                self.plots{i}.past_plot = [];
                self.plots{i}.mfuture_plot = [];
                self.plots{i}.sfuture_plot = [];
                for j=1:self.plots{i}.length
                    nrows = ceil(num_plots/self.ncols);
                    col = floor((cur_plot-1)/nrows);
                    row = cur_plot-1-col*nrows;
                    ind = row*self.ncols+col+1;
                    subplot(ceil(num_plots/self.ncols), self.ncols, ind);
                    if self.plots{i}.length==1
                        ylabel(self.plots{i}.name);
                    else
                        ylabel([self.plots{i}.name '(' num2str(j) ')']);
                    end
                    cur_plot = cur_plot + 1;
                    hold on; grid on;
                    self.plots{i}.past_plot = [self.plots{i}.past_plot plot(0, 0, 'LineWidth', 2)];
                    self.plots{i}.sfuture_plot = [self.plots{i}.sfuture_plot patch(0, 0, 'green', 'EdgeColor', 'none', 'FaceAlpha', 0.5)];
                    self.plots{i}.mfuture_plot = [self.plots{i}.mfuture_plot plot(0, 0, '--b')];
                    self.plots{i}.values = [];
                end
            end
        end
    end
    
end

