function plotTesting(xVariable,yVariable,resultsMean,colour)
%PLOTTESTING Summary of this function goes here
%   Detailed explanation goes here

switch xVariable
    case 'poseSigma';    cols = 1:5;   multipliers = [1 5 10 15 20];
    case 'pointSigma';   cols = 6:10;  multipliers = [1 5 10 15 20];
    case 'planeSigma';   cols = 11:13; multipliers = [1 10 30];
    case 'surfaceSigma'; cols = 14:16; multipliers = [1 10 30];
end

switch yVariable
    case 'initialChiSquaredError';             row = 6;
    case 'finalChiSquaredError';               row = 7;
    case 'constrainedInitialChiSquaredError';  row = 8;
    case 'constrainedFinalChiSquaredError';    row = 9;
    case 'finalNormdX';                        row = 10;
    case 'constrainedFinalNormdX';             row = 11;
    case 'poseTranslationError0';              row = 12;
    case 'poseTranslationError0C';             row = 13;
    case 'poseTranslationErrorN';              row = 14;
    case 'poseTranslationErrorNC';             row = 15;
    case 'poseRotationError0';                 row = 16;
    case 'poseRotationError0C';                row = 17;
    case 'poseRotationErrorN';                 row = 18;
    case 'poseRotationErrorNC';                row = 19;
    case 'allPoseTranslationError0';           row = 20;
    case 'allPoseTranslationError0C';          row = 21;
    case 'allPoseTranslationErrorN';           row = 22;
    case 'allPoseTranslationErrorNC';          row = 23;
    case 'allPoseRotationError0';              row = 24;
    case 'allPoseRotationError0C';             row = 25;
    case 'allPoseRotationErrorN';              row = 26;
    case 'allPoseRotationErrorNC';             row = 27;
    case 'pointError0';                        row = 28;
    case 'pointError0C';                       row = 29;
    case 'pointErrorN';                        row = 30;
    case 'pointErrorNC';                       row = 31;
    case 'allPointError0';                     row = 32;
    case 'allPointError0C';                    row = 33;
    case 'allPointErrorN';                     row = 34;
    case 'allPointErrorNC';                    row = 35;
    case 'planeNormalError0';                  row = 36;
    case 'planeNormalError0C';                 row = 37;
    case 'planeNormalErrorN';                  row = 38;
    case 'planeNormalErrorNC';                 row = 39;
    case 'planeDistanceError0';                row = 40;
    case 'planeDistanceError0C';               row = 41;
    case 'planeDistanceErrorN';                row = 42;
    case 'planeDistanceErrorNC';               row = 43;
    case 'planeFitError0';                     row = 44;
    case 'planeFitError0C';                    row = 45;
    case 'planeFitErrorN';                     row = 46;
    case 'planeFitErrorNC';                    row = 47;
    case 'planeFitErrorEstimate0';             row = 48;
    case 'planeFitErrorEstimate0C';            row = 49;
    case 'planeFitErrorEstimateN';             row = 50;
    case 'planeFitErrorEstimateNC';            row = 51;
    case 'planeFitErrorGroundTruth0';          row = 52;
    case 'planeFitErrorGroundTruth0C';         row = 53;
    case 'planeFitErrorGroundTruthN';          row = 54;
    case 'planeFitErrorGroundTruthNC';         row = 55;
end

camera = 'camera1';
switch camera
    case 'camera1'
        resultsMeanHalf = resultsMean(:,1:16);
    case 'camera2'
        resultsMeanHalf = resultsMean(:,17:32);
end

plot(multipliers,resultsMeanHalf(row,cols),colour);


end

