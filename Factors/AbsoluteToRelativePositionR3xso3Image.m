%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 16/04/18
% Contributors:
%--------------------------------------------------------------------------

function positionsRelative = AbsoluteToRelativePositionR3xso3Image(posesNewFrame,...
    positionsAbsolute, intrinsics, R)

%% converts positions from absolute coords to new reference frame (image plane)
% inputs
    %posesNewFrame = matrix of pose column vectors of new reference frame
    %positionsAbsolute = matrix of absolute position column vectors
% output
    %positionsRelative = matrix of relative position column vectors

for i=1:size(posesNewFrame,2)    
    posesNewFrame(:,i) = transformationMatrixToPose(R\...
        poseToTransformationMatrix(posesNewFrame(:,i))*R);
    positionAbsolute =  R\[positionsAbsolute(:,i);1];
    positionsAbsolute(:,i) =  positionAbsolute(1:3);
end

%to store relative positions
positionsRelative = zeros(size(positionsAbsolute,1)-1,size(positionsAbsolute,2));
%VECTORISE THIS
for i = 1:size(positionsAbsolute,2)
    screwNewFrame = [rot(posesNewFrame(4:6,i)) posesNewFrame(1:3,i);
                     0 0 0 1];
    normalisedPositionRelative = screwNewFrame \ [positionsAbsolute(:,i); 1];
    if numel(intrinsics)==1
        intrinsicMatrix = intrinsics;
    else
        f = intrinsics(1);
        cx = intrinsics(2);
        cy = intrinsics(3);
        intrinsicMatrix = [f 0 cx; 0 f cy; 0 0 1];
    end
    positionRelative = intrinsicMatrix * ...
        (normalisedPositionRelative(1:3,:)/normalisedPositionRelative(3,:));
    positionsRelative(:,i) = positionRelative(1:2);
end

end

