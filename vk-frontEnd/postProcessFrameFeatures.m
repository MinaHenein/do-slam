function frame = postProcessFrameFeatures(frame,maxBackgroundFeaturesPerFrame)


% find number of static background features
nBackgroundFeatures = sum([frame.features.objectId] == -1);

%  if n > maxStaticFeaturesPerFrame, pick strongest maxStaticFeaturesPerFrame
backgroundFeaturesIndx = find([frame.features.objectId] == -1);
assert(nBackgroundFeatures == length(backgroundFeaturesIndx));

if nBackgroundFeatures >  maxBackgroundFeaturesPerFrame
    % delete in a bottom up manner
    for i = length(backgroundFeaturesIndx):-1:maxBackgroundFeaturesPerFrame+1
        frame.features.moving(backgroundFeaturesIndx(i)) = [];
        frame.features.location(backgroundFeaturesIndx(i),:) = [];
        frame.features.objectId(backgroundFeaturesIndx(i)) = [];
        frame.features.originFrame(backgroundFeaturesIndx(i)) = [];
        frame.features.location3D(:,backgroundFeaturesIndx(i)) = [];
    end
end

end