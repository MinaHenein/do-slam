%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 13/09/17
% Contributors:
%--------------------------------------------------------------------------

function posesRelative = AbsoluteToRelativePoseR3xso3GlobalFrame(posesAbsolute,posesNewFrame)
%% converts poses from absolute coords to new reference frame
% inputs
    %posesNewFrame = matrix of pose column vectors of new reference frame
    %posesAbsolute = matrix of absolute pose column vectors, pose = [position; axis]
% output
    %posesRelative = matrix of relative pose column vectors, pose = [position; axis]
    %defined in global frame unlike AbsoluteToRelativePoseR3xso3 where the
    %posesRelative are defined w.r.t to posesNewFrame

%to store relative pose vectors
posesRelative = zeros(size(posesAbsolute));  
%VECTORISE THIS
for i = 1:size(posesAbsolute,2)
    screwAbsolute = [rot(posesAbsolute(4:6,i)) posesAbsolute(1:3,i);
                     0 0 0 1];
    screwNewFrame = [rot(posesNewFrame(4:6,i)) posesNewFrame(1:3,i);
                     0 0 0 1];
    screwRelative =  screwNewFrame/screwAbsolute;
    posesRelative(:,i) = [screwRelative(1:3,4); arot(screwRelative(1:3,1:3))];
end

end



