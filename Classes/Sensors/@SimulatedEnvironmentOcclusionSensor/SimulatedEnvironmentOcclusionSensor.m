%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 30/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef SimulatedEnvironmentOcclusionSensor < SimulatedEnvironmentSensor
    %SimulatedEnvironmentOcclusionSensor is an adaptation of
    %SimulatedEnvironment Sensor with occlusion. It does this by using mesh
    %property of rigid bodies to set visibility for point observations.
    %For the full documentation, see SimulatedEnvironmentSensor.
    
    %% 1. Properties
    properties(GetAccess='protected', SetAccess='protected')
%         pointObservationRelative        
    end
    
    %% 2. Methods
  
    %Declare external methods
    methods(Access = public)
        % point visibility with occlusion
        [visibility,relativePoint] = pointVisibleOcclusion(self,point,meshes,t)
        self = setVisibility(self,config,environment)
        % generate measurements with occlusion
%         generateMeasurements(self,config)
        meshes = generateMeshes(self,environment,t)
    end
    
    methods(Static)
        pointVisibility = checkOcclusion(point, meshes)
    end
    
end

