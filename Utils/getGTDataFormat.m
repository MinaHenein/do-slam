function timeStampedCameraPoseFormat = getGTDataFormat(config)

%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 20/06/17
% Contributors:
%--------------------------------------------------------------------------

% returns the format of the trajectory of the camera from the gt file
% assumes each line consists of the time stamp followed by camera pose
% input: config
% output: format of gt data

switch config.poseRotationRepresentation
    case 'quaternion'
        timeStampedCameraPoseFormat = '%f %f %f %f %f %f %f %f';
    case {'axis-angle','euler angles'}
        timeStampedCameraPoseFormat = '%f %f %f %f %f %f %f';
    case 'rotation matrix'
        timeStampedCameraPoseFormat = '%f %f %f %f %f %f %f %f %f %f %f %f %f';
    otherwise 
        error('Undefined camera pose rotation representation')
end

end