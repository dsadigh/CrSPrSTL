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
         function C = enforce(self, dt, l0, l1, t0, t1)
             C = [];
             for i = 1:numel(self.ps)
                 C = [C self.ps{i}.enforce(dt, l0, l1, t0, t1)]; %#ok<AGROW>
             end
         end
        function C = Tconstraints(self, T, dt, t0)
            C = [];
            for i = 1:numel(self.ps)
                C = [C, self.ps{i}.Tconstraints(T, dt, t0)]; %#ok<AGROW>
            end
        end
        function C = Trobust(self, T, dt, t0)
            C = [];
            for i = 1:numel(self.ps)
                C = [C, self.ps{i}.Trobust(T, dt, t0)]; %#ok<AGROW>
            end
        end
        function C = Fconstraints(self, F, dt, t0)
            C = [];
            Fsum = 0;
            for i = 1:numel(self.ps)
                Fp = binvar(1, numel(F));
                Fsum = Fsum + Fp;
                C = [C, self.ps{i}.Fconstraints(Fp, dt, t0)];  %#ok<AGROW>
            end
            C = [C, F<=Fsum];
        end
        % TODO: for implementing adversarial agents
        function C = Frobust(self, F, dt, t0)
            C = [];
            Fps = {};
            for i = 1:numel(self.ps)
                Fp = sdpvar(1, numel(F));
                Fps = [Fmax {Fp}]; %#ok<AGROW>
                C = [C, self.ps{i}.Frobust(Fp, dt, t0)]; %#ok<AGROW>
            end
            C = [C, max_ge(Fps, F)];
        end
        %function C = forced_constraints(self, dt, L_start, L_end, t_start, t_end)
        %    C = [];
        %    for i = 1:numel(self.ps)
        %        C = [C, self.ps{i}.forced_constraints(dt, L_start, L_end, t_start, t_end)]; %#ok<AGROW>
        %    end
        %end
    end
end