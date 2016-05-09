classdef (Abstract) Predicate
    % Class: Predicate: Predicate has a set of True and False constraints: 
    %        Tconstraints = True => predicate = True
    %        Fconstraints = True => predicate = False
    % methods: Defining operations on predicates.
    %          functions: and, or, not, implies, eventually, always, until
    

    methods (Abstract)
        C = Tconstraints(self, T, dt, t0, varargin)
        C = Fconstraints(self, F, dt, t0, varargin)
    end
    
    methods
        function C = enforce(self, dt, l0, l1, t0, t1, varargin)
            l1 = l0+round((l1-l0)/dt)*dt;
            t0 = max(t0, l0);
            t1 = min(t1, l1);
            a = round((t0-l0)/dt)+1;
            b = round((t1-l0)/dt)+1;
            T = binvar(1, round((l1-l0)/dt)+1);
            C = self.Tconstraints(T, dt, l0, varargin{:});
            C = [C T(a:b)>=1];
        end
        function result = and(varargin)
            result = AndPredicate(varargin{:});
        end
        function result = or(varargin)
            for i = 1:numel(varargin)
                varargin{i} = NotPredicate(varargin{i});
            end
            result = NotPredicate(AndPredicate(varargin{:}));
        end
        function result = not(p)
            result = NotPredicate(p);
        end
        function result = implies(p, q)
            result = NotPredicate(AndPredicate(p, NotPredicate(q)));
        end
        function result = eventually(p, varargin)
            result = NotPredicate(AlwaysPredicate(NotPredicate(p), varargin{:}));
        end
        function result = always(p, varargin)
            result = AlwaysPredicate(p, varargin{:});
        end
        function result = until(p, q, t1, t2)
            switch(nargin)
                case 2
                    result = UntimedUntilPredicate(p, q);
                case 4
                    result = AndPredicate(AlwaysPredicate(p, 0, t1), FuturePredicate(q, t1, t2), UntimedUntilPredicate(p, q, t1));
                otherwise
                    error('Invalid number of arguments')
            end
        end
    end
end