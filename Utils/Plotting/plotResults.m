function plotResults(results)

%% mean
resultsMean = zeros(55,32);
for i = 1:32
    resultsMean(:,i) = sum(results(:,mapping(i,10)),2)/10;
end

xVariables = {'poseSigma','pointSigma','planeSigma','surfaceSigma'};
yVariables = {'initialChiSquaredError','finalChiSquaredError',...,
    'constrainedInitialChiSquaredError','constrainedFinalChiSquaredError',...,
    'finalNormdX','constrainedFinalNormdX',...
    'poseTranslationError0','poseTranslationError0C',...
    'poseTranslationErrorN','poseTranslationErrorNC',...
    'poseRotationError0','poseRotationError0C',...
    'poseRotationErrorN','poseRotationErrorNC',...
    'allPoseTranslationError0','allPoseTranslationError0C',...
    'allPoseTranslationErrorN','allPoseTranslationErrorNC',...
    'allPoseRotationError0','allPoseRotationError0C',...
    'allPoseRotationErrorN','allPoseRotationErrorNC',...
    'pointError0','pointError0C','pointErrorN','pointErrorNC',...
    'allPointError0','allPointError0C','allPointErrorN','allPointErrorNC',...
    'planeNormalError0','planeNormalError0C',...
    'planeNormalErrorN','planeNormalErrorNC',...
    'planeDistanceError0','planeDistanceError0C',...
    'planeDistanceErrorN','planeDistanceErrorNC',...
    'planeFitError0','planeFitError0C','planeFitErrorN','planeFitErrorNC',...
    'planeFitErrorEstimate0','planeFitErrorEstimate0C',...
    'planeFitErrorEstimateN','planeFitErrorEstimateNC',...
    'planeFitErrorGroundTruth0','planeFitErrorGroundTruth0C',...
    'planeFitErrorGroundTruthN','planeFitErrorGroundTruthNC'};

xVars = [1:4];
yVars = [5 7  9 11 12 15 16 19 20 23 24 27 28 31 32 35 36 39 40 43 44 47 48 51 52;
    6 8 10 13 14 17 18 21 22 25 26 29 30 33 34 37 38 41 42 45 46 49 50 53 54] - 4;

for i = 1:size(xVars,2)
    for j = 1:size(yVars,2)
        figure
        subplot(2,1,1)
        plotTesting(xVariables{xVars(i)},yVariables{yVars(1,j)},resultsMean,'b.-')
        xlabel(strcat(xVariables{xVars(i)},' multiplier'))
        ylabel(yVariables{yVars(1,j)})
        title(strcat(xVariables{xVars(i)},' vs',' ',yVariables{yVars(1,j)}))
        
        subplot(2,1,2)
        plotTesting(xVariables{xVars(i)},yVariables{yVars(2,j)},resultsMean,'b.-')
        xlabel(strcat(xVariables{xVars(i)},' multiplier'))
        ylabel(yVariables{yVars(2,j)})
        title(strcat(xVariables{xVars(i)},' vs',' ',yVariables{yVars(2,j)}))
    end
end

x = 1:320;
figure
subplot(2,1,1)
plot(x,results(2,:))
xlabel('number of trials')
ylabel('solve time in case of no constraints')
title('Number of trials vs solving time with no added constraints')
subplot(2,1,2)
plot(x,results(3,:))
xlabel('number of trials')
ylabel('solve time in case of added constraints')
title('Number of trials vs solving time with added constraints')

figure
subplot(2,1,1)
plot(x,results(4,:))
xlabel('number of trials')
ylabel('number of iterations in case of no constraints')
title('Number of trials vs number of iterations with no added constraints')
subplot(2,1,2)
plot(x,results(5,:))
xlabel('number of trials')
ylabel('number of iterations in case of added constraints')
title('Number of trials vs number of iterations with added constraints')


figlist=findobj('Type','figure');
for i=1:numel(figlist)
    saveas(figlist(i),fullfile('/home/mina/workspace/src/Git/dynamicSLAM/Results',['figure' num2str(i) '.jpg']));
end