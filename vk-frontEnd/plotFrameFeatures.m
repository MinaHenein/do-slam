function plotFrameFeatures(frame,rgbI)

frameName = strcat(repmat('0',1,5-numel(num2str(frame.number))),num2str(frame.number),'.png');
rgbIm = imread(strcat(rgbI,frameName));

figure;
imshow(rgbIm);
hold on
for i = 1:length(frame.features.location)
    featureLocation = frame.features.location(i,:);
    featureOriginFrame = frame.features.originFrame(i);
    nFramesForward = frame.number - featureOriginFrame;
    switch nFramesForward
        case 0; color = rgb('light blue');
        case 1; color = rgb('blue');
        case 2; color = rgb('dark blue');
        case 3; color = rgb('light red');
        case 4; color = rgb('red');
        case 5; color = rgb('dark red');
        otherwise; color = rgb('magenta');
    end
    scatter(featureLocation(1,1),featureLocation(1,2),15,'MarkerEdgeColor',color,'MarkerFaceColor',color);
    hold on

end


end