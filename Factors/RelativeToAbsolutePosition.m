function positionsAbsolute = RelativeToAbsolutePosition(posesOldFrame, positionsRelative)
%% converts positions from absolute coords to new reference frame
% inputs
    %poseNewFrame = matrix of pose column vectors of new reference frame
    %positionsRelative = matrix of relative position column vectors
% output
    %positionsAbsolute = matrix of absolute position column vectors

%to store absolute positions
positionsAbsolute = zeros(size(positionsRelative));
%VECTORISE THIS
for i = 1:size(positionsRelative,2)
    screwOldFrame = [rot(posesOldFrame(4:6,i)) posesOldFrame(1:3,i);
                     0 0 0 1];
    normalisedPositionAbsolute = screwOldFrame*[positionsRelative(:,i); 1];
    positionsAbsolute(:,i) = normalisedPositionAbsolute(1:3,:);
end

end

