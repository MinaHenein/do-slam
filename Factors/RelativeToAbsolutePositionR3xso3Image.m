%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 16/04/18
% Contributors:
%--------------------------------------------------------------------------

function positionsAbsolute = RelativeToAbsolutePositionR3xso3Image(posesOldFrame, positionsRelative,intrinsics)
%% converts positions from relative coords in image frame to absolute coords
% inputs
    %poseOldFrame = matrix of pose column vectors of old reference frame
    %positionsRelative = matrix of relative position column vectors
% output
    %positionsAbsolute = matrix of absolute position column vectors

%to store absolute positions
positionsAbsolute = zeros(size(positionsRelative));
%VECTORISE THIS
for i = 1:size(positionsRelative,2)
    screwOldFrame = [rot(posesOldFrame(4:6,i)) posesOldFrame(1:3,i);
                     0 0 0 1];
    % intrinsics here is just a scalar value - focal length
    normalisedPositionAbsolute = screwOldFrame*[positionsRelative(:,i)/intrinsics; 1];
    positionsAbsolute(:,i) = normalisedPositionAbsolute(1:3,:);
end

end

