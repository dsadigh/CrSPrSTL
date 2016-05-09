classdef AndPredicate < Predicate
    % class: AndPredicate
    % properties: ps: a set of predicates 
    % methods: Tconstraints, Fconstraints: Defines yalmip constraints for
    %           satisfaction of AndPredicate.
    
    properties
        ps
    end
    methods
        function self = AndPredicate(varargin)
            self.ps = varargin;
        end
         function C = enforce(self, dt, l0, l1, t0, t1, varargin)
             C = [];
             for i = 1:numel(self.ps)
                 C = [C self.ps{i}.enforce(dt, l0, l1, t0, t1, varargin{:})]; %#ok<AGROW>
             end
         end
        function C = Tconstraints(self, T, dt, t0, varargin)
            C = [];
            for i = 1:numel(self.ps)
                C = [C, self.ps{i}.Tconstraints(T, dt, t0, varargin{:})]; %#ok<AGROW>
            end
        end
        function C = Fconstraints(self, F, dt, t0, varargin)
            C = [];
            Fsum = 0;
            for i = 1:numel(self.ps)
                Fp = binvar(1, numel(F));
                Fsum = Fsum + Fp;
                C = [C, self.ps{i}.Fconstraints(Fp, dt, t0, varargin{:})];  %#ok<AGROW>
            end
            C = [C, F<=Fsum];
        end
    end
end