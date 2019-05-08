function depthFromZEDStereo(video,startFrame,endFrame)

framesPath = strcat('/media/mina/JunHDD/Dynamic SLAM Data/',video,'/'); 

for i=startFrame:endFrame
    
    nDigits = numel(num2str(i));
    fileName = strcat(framesPath,'img_',repmat('0',[1,9-nDigits]),num2str(i),'.ppm');
    I = imread(fileName);
    
    frameLeftRect = I(1:size(I,1),1:size(I,2)/2,:);
    frameRightRect = I(1:size(I,1),size(I,2)/2+1:size(I,2),:);
    
    frameLeftGrayRect  = rgb2gray(frameLeftRect);
    frameRightGrayRect = rgb2gray(frameRightRect);
    
    disparityMap = disparitySGM(frameLeftGrayRect, frameRightGrayRect);
    imwrite(disparityMap,strcat(fileName(1:end-4),'_depth.jpg'));
    
end
