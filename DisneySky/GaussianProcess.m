function GaussianProcess(trainInput, trainMocap, testInput, testMocap, jointInd)

[finalX, removalInds]  = parseMocap(trainMocap, jointInd)

[finalXTest, removalIndsTest]  = parseMocap(testMocap, jointInd)

x = trainInput/1950;

x = x(:,2:40);

x = x(removalInds,:);

xtest = testInput/1950;

xtest = xtest(:,2:40);

predictedMeans = [];

for ourVar = 1:3
y = finalX(:,ourVar);
  meanfunc = {@meanConst}; hyp.mean = [0];
  covfunc = {@covSEiso}; ell = 1/4; sf = 1; hyp.cov = log([ell; sf]);
  likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);
 
  [m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, xtest);
  predictedMeans = [predictedMeans m]
end

clear figure;

plot3(predictedMeans(:,1), predictedMeans(:,2), predictedMeans(:,3));
hold on;
plot3(finalX(:,1), finalX(:,2), finalX(:,3), 'Color', 'red');
plot3(finalXTest(:,1), finalXTest(:,2), finalXTest(:,3), 'Color', 'green');

end

function [finalX, removalInds]  = parseMocap(mocap, j)

finalX = [];

removalInds = [];

Joint = mocap{j,1}


T = mocap(:,1);
T = cell2mat(T);

ourLines = find(T == Joint);

X = []
Y = []
Z = []

for i = 2:size(ourLines, 1)
    
    if(i > 2 && abs(X{end} - mocap{ourLines(i),2}) > 20)
        continue;
    end
    
    if(i > 2 && abs(Y{end} - mocap{ourLines(i),3}) > 20)

        continue;
        
    end
    
    if(i > 2 && abs(Z{end} - mocap{ourLines(i),4}) > 20)
        continue;
    end
    removalInds = [removalInds; i - 1];
    
    X = [X; {mocap{ourLines(i),2}}];
    Y = [Y; {mocap{ourLines(i),3}}];
    Z = [Z; {mocap{ourLines(i),4}}];
end


finalX = [finalX cell2mat(X) cell2mat(Y) cell2mat(Z)];

end

