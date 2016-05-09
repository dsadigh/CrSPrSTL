classdef Sample < handle
    properties
        values
    end
    
    methods
        function self = Sample(sample)
            self.values = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            if nargin>=1
                self.values = [self.values; sample.values];
            end
        end
        function overwrite(self, sample)
            self.values = [self.values; sample.values];
        end
        function set_value(self, x, val)
            for i=1:numel(x)
                self.values(depends(x(i))) = val(i);
            end
        end
        function result = filled(self, C)
            vars = depends(C);
            global VARS_MAP;
            %vars = vars(VARS_MAP.isKey(num2cell(vars)));
            for xnum = vars(VARS_MAP.isKey(num2cell(vars)) & ~self.values.isKey(num2cell(vars)))
                gen = VARS_MAP(xnum);
                self.values(xnum) = gen();
            end
            vars = vars(self.values.isKey(num2cell(vars)));
            vals = values(self.values, num2cell(vars)); %#ok<CPROPLC>
            vals = [vals{:}];
            if ~isempty(vars)
                result = replace(C, recover(vars), vals);
            else
                result = C;
            end
        end
    end
end

