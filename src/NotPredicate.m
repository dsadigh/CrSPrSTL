classdef NotPredicate < Predicate
    % class: NotPredicate
    % properties: p: predicate
    % methods: Tconstraints, Fconstraints: Defines yalmip constraints for
    %           satisfaction of NotPredicate.

    properties
        p
    end
    
    methods
        function self = NotPredicate(p)
            self.p = p;
        end
        function C = Tconstraints(self, T, dt, t0)
            C = self.p.Fconstraints(T, dt, t0);
        end
        function C = Trobust(self, T, dt, t0)
            C = self.p.Frobust(T, dt, t0);
        end
        function C = Fconstraints(self, F, dt, t0)
            C = self.p.Tconstraints(F, dt, t0);
        end
        function C = Frobust(self, F, dt, t0)
            C = self.p.Trobust(F, dt, t0);
        end
        %function C = forced_constraints(varargin) %#ok<STOUT>
        %    error('Forced constraints cannot be implemented for not!');
        %end
    end
    
end

