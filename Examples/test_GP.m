%% test_GP.m: Testing LinearGP.m and defining a linear classifier
clear all
clf;

%Define our workspace (0, 0) to (1,1)
RNG = 0:0.01:1; 
Xdim1 = repmat(RNG, length(RNG), 1);
Xdim2 = repmat(RNG', 1, length(RNG));

%Define the ground truth obstactle
Ytrue = (Xdim2 <= 0.3) - (Xdim2 > 0.3);

%Visualize the Ground Truth
subplot(1, 3, 1);
imagesc(Ytrue); axis equal off;

%Sample Ground Truth - Each Row is one Input Location
Xtraining = [0.2 0.3; 0.8 0.3; 0.2 0.7; 0.7 0.6];
Ytraining = zeros(size(Xtraining, 1), 1);
%Visualize the Training Points;
hold on;
for i = 1:size(Xtraining, 1)
    Ind_fun = (Xdim1 == Xtraining(i, 1)).*(Xdim2 == Xtraining(i, 2));
    [xind, yind] = find(Ind_fun == 1);
    scatter(yind, xind, 'filled', 'y');
    
    Ytraining(i) = Ytrue(xind, yind);
end

%Compute the GP Mean and Covariance on the predictor W:
Xtraining = [Xtraining ones(length(Ytraining), 1)];
[m, S] = linearGP(Xtraining, Ytraining, 1);

%Compute Predictions - means and covariances
Ymean = zeros(length(RNG), length(RNG));
Ycov = zeros(length(RNG), length(RNG));
for i = 1:length(RNG)
    for j = 1:length(RNG)
        x_star = [RNG(j) RNG(i) 1];
        Ymean(i, j) = x_star*m;
        Ycov(i, j) = x_star*S*x_star';
    end
end


%Visualize the Mean Function
subplot(1, 3, 2);
imagesc(Ymean > 0); axis equal off;

%Visualize the Covariance Function
subplot(1, 3, 3);
imagesc(Ycov); axis equal off;