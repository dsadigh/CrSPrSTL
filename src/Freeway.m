classdef Freeway < System
    % class: Vehicle: Subclass of system used for defining the nonlinear 
    %                   dynamics of a point-mass model of a vehicle
    % methods: plotter, Vehicle
    
    properties
        f
        r
    end
    
    methods
        function plotter_hook(self, plotter)
            plotter.add_signal('f', self.f);
        end
        
        function self = Freeway(dt, f0, beta, v, w, fbar, nbar, rbar, dmean, dstd)
            self@System(dt);
            N = length(nbar);
            self.set_dimensions(2*N, N);
            [x, r] = self.signals();
            bhat = 1-beta;
            n = x(:, 1:N);
            l = x(:, N+1:2*N);
            f = Signal(dt, N);
            self.f = f;
            d = Signal(dt, N);
            p1 = P(@(t, dt) n(t+dt) == n(t) - f(t)./bhat + r(t) + [f0; f(t, 1:N-1)]);
            %p2 = P(@(t, dt) l(t+dt) == l(t) - r(t) + d(t));
            p2 = P(@(t, dt) l(t+dt) == l(t) + r(t));
            
            function constraint = l_constraint(x0, t0, dt)
                constraint = AndPredicate();
                for j=1:N
                    %constraint = Pr(@(t, dt) l(t, j)<=dmean(j)*(t/dt))>=0.8;
                    constraint = constraint & always(Pr(@(t, dt) l(t, j) <= dmean(j)*(t0/dt) + normal(dmean(j)*(t-t0)/dt, dstd(j)^2*(t-t0)/dt))>=0.8);
                end
            end
            
            self.add_dyn_constraint(@l_constraint);
            
            p3 = P(@(t, dt) f(t, N) == min(bhat(N) * v(N) * (n(t, N)+r(t, N)), fbar(N)));
            for i=1:N-1
                p3 = and(p3, P(@(t, dt) f(t, i) == min(bhat(i) * w(i) * (nbar(i+1)-n(t, i+1)-r(t, i+1)), ...
                min(bhat(i) * v(i) * (n(t, i)+r(t, i)), fbar(i)))));
            end
            %p3 = P(@(t, dt) f(t) == min(bhat .* v .* (n(t)+r(t)), fbar, bhat .* w .* [inf; subsref(nbar - n(t) - r(t), substruct('()', {1:N-1}))]));
            %p4 = P(@(t, dt) d(t) == 2);
            p = always(and(p1, p2, p3));
            dyn = ConstraintDynamics(x, r, p);
            self.set_dynamics(dyn);
            
            self.add_constraint(P(@(t, dt) x(0)==0));
            %self.add_constraint(always(P(@(t, dt) l(t)>=0)));
            self.add_constraint(always(P(@(t, dt) r(t)>=0)));
            self.add_constraint(always(P(@(t, dt) r(t)<=rbar)));
        end
    end
    
end