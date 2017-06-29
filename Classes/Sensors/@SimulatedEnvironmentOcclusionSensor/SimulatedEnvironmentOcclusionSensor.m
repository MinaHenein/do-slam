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
    properties
    end
    
    %% 2. Methods
  
    % Add environment
    methods(Access = public)
    end
    
    %Declare external methods
    methods(Access = public)
        % point visibility with occlusion
        [visibility,relativePoint] = pointVisibleOcclusion(self,point,t)
        % generate measurements with occlusion
        generateMeasurementsOcclusion(self,config)
        meshes = generateMeshes(self,t)
    end
    
end

