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
        
        function C = enforce(self, dt, l0, l1, t0, t1, varargin)
            C = self.p.enforce(dt, l0, l1, t0+self.t1, t1+self.t2, varargin{:});
        end
        
        function C = Tconstraints(self, T, dt, t0, varargin)
            a = round(self.t1/dt);
            b = round(self.t2/dt);
            Tp = binvar(1, numel(T));
            C = self.p.Tconstraints(Tp, dt, t0, varargin{:});
            for t = a:min(b, numel(T)-1)
                C = [C, T(1:end)<=[Tp(1+t:end), ones(1, t)]]; %#ok<AGROW>
            end
        end
        
        function C = Fconstraints(self, F, dt, t0, varargin)
            a = round(self.t1/dt);
            b = round(self.t2/dt);
            Fp = binvar(1, numel(F));
            C = self.p.Fconstraints(Fp, dt, t0, varargin{:});
            Fsum = 0;
            for t = a:min(b, numel(F))
                % Being optimistic about future
                Fsum = Fsum + [Fp(1+t:end), ones(1, t)];
            end
            C = [C, F<=Fsum];
        end
    end
    
end

