clc; clear all; close all;

%for i=1:20
I = imread('10.jpg');
% I_k = rgb2gray(I(:,:,1:3));
for ii=1:1:3
    I_k = I(:,:,ii);
%     figure; imshow(I_k); pause;
    regions = detectMSERFeatures(I_k,'ThresholdDelta',5/255,...
        'RegionAreaRange', [200 3000],...
        'MaxAreaVariation', 0.5);
    figure;
    imshow(I); hold on; plot(regions,'showPixelList',true,'showEllipses',false)
    pause;
    
end
