%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 11/08/17
% based on Montiel Abello's RelativeToAbsolutePositionR3xso3 function - 
% montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function positionsAbsolute = RelativeToAbsolutePositionR3xso3Normalised(posesOldFrame, positionsRelative)
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
    normalisedPositionAbsolute = screwOldFrame* positionsRelative(:,i);
    positionsAbsolute(:,i) = normalisedPositionAbsolute(1:4,:);
end

end

