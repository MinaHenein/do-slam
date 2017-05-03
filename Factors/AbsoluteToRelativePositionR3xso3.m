function positionsRelative = AbsoluteToRelativePositionR3xso3(posesNewFrame, positionsAbsolute)
%% converts positions from absolute coords to new reference frame
% inputs
    %posesNewFrame = matrix of pose column vectors of new reference frame
    %positionsAbsolute = matrix of absolute position column vectors
% output
    %positionsRelative = matrix of relative position column vectors

%to store relative positions
positionsRelative = zeros(size(positionsAbsolute));
%VECTORISE THIS
for i = 1:size(positionsAbsolute,2)
    screwNewFrame = [rot(posesNewFrame(4:6,i)) posesNewFrame(1:3,i);
                     0 0 0 1];
    normalisedPositionRelative = screwNewFrame \ [positionsAbsolute(:,i); 1];
    positionsRelative(:,i) = normalisedPositionRelative(1:3,:);
end

end

