function result = tracked_var(gen, varargin)

    global VARS_MAP;
    if isempty(VARS_MAP)
        VARS_MAP = containers.Map('KeyType', 'int64', 'ValueType', 'any');
    end
    
    if ischar(gen)
        if strcmp(gen, 'stochastic')
            gen = @() sdpvar();
        else
            dist = makedist(gen);
            gen = @() random(dist);
        end
    elseif ~isa(gen, 'function_handle')
        gen = @() random(gen);
    end 

    if numel(varargin)==1
        result = sdpvar(varargin{1}, 1);
    else
        result = sdpvar(varargin{:});
    end
    
    for xnum = depends(result)
        VARS_MAP(xnum) = gen;
    end
end