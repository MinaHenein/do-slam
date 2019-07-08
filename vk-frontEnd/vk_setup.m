% variables
sequence = '0001';
variation = 'clone';
imageRange = 121:154;
nFeaturesPerFrame = 600; % number of features per frame
maxBackgroundFeaturesPerFrame = 200; % max number of static background features per frame
nFeaturesPerObject = 100; % number of features per object

noiseArray = [0.1 0.1 0.1 0.05 0.05 0.05 0.04 0.04 0.04];
applyNoise = 1;
rng(12);

% setup
cd /home/mina/workspace/src/Git/do-slam
dir = '/media/mina/Data/mina/Downloads/Virtual_KITTI/';
K = [725, 0, 620.5; 0, 725, 187.0; 0, 0, 1];
% directories
rgbDir = 'vkitti_1.3.1_rgb/';
depthDir = 'vkitti_1.3.1_depthgt/';
objSegDir = 'vkitti_1.3.1_scenegt/';
motDir = 'vkitti_1.3.1_motgt/';
extrinsicsDir = 'vkitti_1.3.1_extrinsicsgt/';
% data
rgbI = strcat(dir,rgbDir,sequence,'/',variation,'/');
depthI = strcat(dir,depthDir,sequence,'/',variation,'/');
maskI = strcat(dir,objSegDir,sequence,'/',variation,'/');
motFile = strcat(dir,motDir,sequence,'_',variation,'.txt');
extrinsicsFile = strcat(dir,extrinsicsDir,sequence,'_',variation,'.txt');

% pre-processing
fprintf('Preprocessing data ...\n')
cameraPoses = preprocessExtrinsics(extrinsicsFile,imageRange);
odometry = extractOdometry(cameraPoses);
% feature extraction
fprintf('Feature extraction and tracking ...\n')
[frames,globalFeatures] = featureExtractionTracking(imageRange,K,rgbI,depthI,maskI,...
    motFile,cameraPoses,nFeaturesPerFrame,nFeaturesPerObject,maxBackgroundFeaturesPerFrame);
% graph files
fprintf('Writing graph files ...\n')
[globalCamerasGraphFileIndx, globalFeaturesGraphFileIndx, globalObjectsGraphFileIndx] = ...
    writeGTGraphFile(frames, globalFeatures, imageRange, sequence);
writeMeasGraphFile(frames, globalFeatures, imageRange, sequence,...
    globalCamerasGraphFileIndx, globalFeaturesGraphFileIndx, globalObjectsGraphFileIndx, noiseArray, applyNoise);
