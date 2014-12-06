


finalX = [];
x = ourInput/1950;
x = [x;x(800,:)];

for j = 2:5
Joint = withMocap1{j,1}


T = withMocap1(:,1);
T = cell2mat(T);

ourLines = find(T == Joint);

X = []
Y = []
Z = []

for i = 2:size(ourLines, 1)
    X = [X; {withMocap1{ourLines(i),2}}];
    Y = [Y; {withMocap1{ourLines(i),3}}];
    Z = [Z; {withMocap1{ourLines(i),4}}];
end

finalX = [finalX cell2mat(X) cell2mat(Y) cell2mat(Z)];

end

x(finalX(:,1) < 0,:) = []
finalX(finalX(:,1) < 0,:) = []

x(finalX(:,2) < 0,:) = []
finalX(finalX(:,2) < 0,:) = []

x(finalX(:,3) < 0,:) = []
finalX(finalX(:,3) < 0,:) = []

x = x(:,2:40);

%theMean = mean(x);
%x = x - repmat(theMean, size(x,1), 1);

%theSigma = x' * x / size(x,2);

%[U,S,V] = svd(theSigma);

%x = x * U *diag(1./sqrt(diag(S) + 0.001));

x = x(:,1:9);

%P = pca(x);

%P = P(:,1:7);

%x = x*P;

xtest = withMocap2/1950;
xtest = withMocap3/1950;

xtest = xtest(:,2:40);

%xtest = xtest - repmat(theMean, size(xtest,1), 1);

%theSigma = xtest' * xtest / size(xtest,2);

%[U,S,V] = svd(theSigma);

%xtest = xtest * U *diag(1./sqrt(diag(S) + 0.001));

xtest = xtest(:,1:9);

predictedMeans = [];

for ourVar = 1:3
y = finalX(:,ourVar);
  meanfunc = {@meanConst}; hyp.mean = [mean(y)];
  covfunc = {@covSEiso}; ell = 2.5; sf = 1; hyp.cov = log([ell; sf]);
  likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);
 
  [m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, xtest);
  predictedMeans = [predictedMeans m]
end

mean(norm(y - finalX(:, 1:3)))

plot3(predictedMeans(:,1), predictedMeans(:,2), predictedMeans(:,3));
plot3(finalX(:,1), finalX(:,2), finalX(:,3));

