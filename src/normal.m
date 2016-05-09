function result = normal(mu, sigma)
    % function: normal: Returns a normal variable based on standard normal
    % input: mu, sigma: mean and variance
    % output: Normal random variable with mu and sigma
    
    if nargin==0
        mu = 0.;
        sigma = 1.;
    elseif nargin==1
        sigma = eye(length(mu));
    end
    
    T = cholcov(sigma);
    if size(T, 1)==0
        result = mu;
    else
        result = T'*standard_normal(size(T, 1));
        if size(result)~=size(mu)
            result = mu + result';
        else
            result = mu + result;
        end
    end
end