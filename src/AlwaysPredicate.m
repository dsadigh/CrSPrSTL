classdef AlwaysPredicate < Predicate
    % class: AlwaysPredicate: G_[t1,t2] p, Gp = G_[0,inf] p
    % properties: predicate, time bounds [t1,t2]
    % methods: Tconstraints, Fconstraints: Defines yalmip constraints for
    %           satisfaction of AlwaysPredicates.

    properties
        p
        t1
        t2
    end
    
    methods
        function self = AlwaysPredicate(p, t1, t2)
            self.p = p;
            switch (nargin)
                case 3
                    self.t1 = t1;
                    self.t2 = t2;
                case 1
                    self.t1 = 0;
                    self.t2 = inf;
                otherwise
                    error('Invalid number of parameters')
            end
        end
        
        function C = enforce(self, dt, l0, l1, t0, t1)
            C = self.p.enforce(dt, l0, l1, t0+self.t1, t1+self.t2);
        end
        
        function C = Tconstraints(self, T, dt, t0)
            a = round(self.t1/dt);
            b = round(self.t2/dt);
            Tp = binvar(1, numel(T));
            C = self.p.Tconstraints(Tp, dt, t0);
            for t = a:min(b, numel(T)-1)
                C = [C, T(1:end)<=[Tp(1+t:end), ones(1, t)]]; %#ok<AGROW>
            end
        end
        
        function C = Trobust(self, T, dt, t0)
            a = round(self.t1/dt);
            b = round(self.t2/dt);
            Tp = sdpvar(1, numel(T));
            Tps = {};
            C = self.p.Trobust(Tp, dt, t0);
            for t = a:min(b, numel(T)-1)
                Tps = [Tps {Tp(1+t:end)}]; %#ok<AGROW>
            end
            C = [C, min_ge(Tps, T)];
        end
        
        function C = Fconstraints(self, F, dt, t0)
            a = round(self.t1/dt);
            b = round(self.t2/dt);
            Fp = binvar(1, numel(F));
            C = self.p.Fconstraints(Fp, dt, t0);
            Fsum = 0;
            for t = a:min(b, numel(F))
                % Being optimistic about future
                Fsum = Fsum + [Fp(1+t:end), ones(1, t)];
            end
            C = [C, F<=Fsum];
        end
        % TODO: For impleenting adversarial agents
        function C = Frobust(self, F, dt, t0)
            a = round(self.t1/dt);
            b = round(self.t2/dt);
            Fp = sdpvar(1, numel(F));
            C = self.p.Frobust(Fp, dt, t0);
            Fps = {};
            for t = a:min(b, numel(F))
                Fps = [Fps {Fp(1+t:end)}];
            end
            C = [C, max_ge(Fps, F)];
        end
        
        %function C = forced_constraints(self, dt, L_start, L_end, t_start, t_end)
        %    C = self.p.forced_constraints(dt, L_start, L_end, t_start+self.t1, t_end+self.t2);
        %end
    end
    
end

