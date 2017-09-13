%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 15/08/17
% Contributors:
%--------------------------------------------------------------------------

function posesAbsolute = RelativeToAbsolutePoseR3xso3GlobalFrame(posesOldFrame, posesRelative)
%% converts poses from relative coords to absolute
% inputs
    %poseOldFrame  = matrix of pose column vectors of old reference frame
    %posesRelative = matrix of relative pose column vectors, pose = [position; axis]
    %defined in global frame unlike RelativeToAbsolutePoseR3xso3 where the
    %relative poses are defined w.r.t to poseOldFrame
% output
    %posesAbsolute = matrix of absolute pose column vectors, pose = [position; axis]
    
%to store absolute pose vectors
posesAbsolute = zeros(size(posesRelative));                    
%VECTORISE THIS
for i = 1:size(posesRelative,2)
    screwRelative = [rot(posesRelative(4:6,i)) posesRelative(1:3,i);
                    0 0 0 1];
    screwOldFrame = [rot(posesOldFrame(4:6,i)) posesOldFrame(1:3,i);
                    0 0 0 1];
    screwAbsolute = screwRelative*screwOldFrame;
    posesAbsolute(:,i) = [screwAbsolute(1:3,4); arot(screwAbsolute(1:3,1:3))];
end

end
