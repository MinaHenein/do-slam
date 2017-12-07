%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/12/17
% Contributors:
%--------------------------------------------------------------------------

classdef ConstantAccelerationDiscretePoseTrajectory < DiscretePoseTrajectory
    %CONSTANTACCELERATIONDISCRETEPOSETRAJECTORY is a child class trajectory of
    %Disrete Pose trajectory. It applies the motion input for every time
    %step t provided, parameterised as R3xso3, logSE3 or SE(3). As t is
    %discrete only poses at times 't' can be requested with the 'get' method.
    %Note that at this time, to correctly instantiate SE(3) or log(SE(3) poses
    %you must not provide inputs as GP_Pose type.
   
    %% 1. Properties
    properties(Hidden) % inherited from DiscretePoseTrajectory
    end
    
    properties(Dependent)
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = ConstantAccelerationDiscretePoseTrajectory(t,initialPose,motion,acceleration,property)
            switch nargin
                case 0 %allow preallocation
                otherwise
                    % assert t unique
                    assert(numel(t)==numel(unique(t)),'Error: t cannot contain duplicates')
                    t = sort(t); % sorts t values, ideally these are linspace anyway
                    nPoses = numel(t);
                    
                    poses(nPoses) = GP_Pose();
                    % convert poses to GP_Pose
                    if ~isa(initialPose,'GP_Pose')
                        poses(1) = GP_Pose(initialPose,property); % first pose
                    else
                        poses(1) = initialPose;
                    end                   
                    
                    % only uses motion Pose if parameterised
                    for i = 2:nPoses
                        motion = motion+acceleration*t(i);
                        if isa(motion,'GP_Pose')
                            poses(i) = poses(i-1).RelativeToAbsolutePose(motion);
                        else
                            switch property
                                case 'R3xso3'
                                    poses(i) = GP_Pose(RelativeToAbsolutePoseR3xso3(poses(i-1).get('R3xso3Pose'),motion),'R3xso3');
                                case 'logSE3'
                                    poses(i) = GP_Pose(Relative2AbsoluteSE3(poses(i-1).get('logSE3Pose'),motion),'logSE3');
                                case 'SE3'
                                    assert(size(motion)==[4,4],'Motion input is not an SE(3) matrix.')
                                    poses(i) = GP_Pose(Relative2AbsoluteSE3(poses(i-1).get('SE3'),motion),'SE3');                                
                            end
                        end
                    end
                                       
                    % set properties
                    self.poses = poses;
                    self.t     = t;
            end
                    
        end
        
    end
    
   
end

