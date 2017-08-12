%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 11/08/17
% based on Montiel Abello's AbsoluteToRelativePositionR3xso3 function - 
% montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function positionsRelative = AbsoluteToRelativePositionR3xso3Normalised(posesNewFrame, positionsAbsolute)
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
    normalisedPositionRelative = screwNewFrame \ positionsAbsolute(:,i);
    positionsRelative(:,i) = normalisedPositionRelative(1:4,:);
end

end

