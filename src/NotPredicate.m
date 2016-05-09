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
        function C = Tconstraints(self, T, dt, t0, varargin)
            C = self.p.Fconstraints(T, dt, t0, varargin{:});
        end
        function C = Fconstraints(self, F, dt, t0, varargin)
            C = self.p.Tconstraints(F, dt, t0, varargin{:});
        end
    end
    
end

