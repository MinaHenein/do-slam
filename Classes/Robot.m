classdef Robot < matlab.mixin.Copyable & handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        trajectory %trajectory class instance
        sensors    %array of sensor class instances
        relativeSensorPoses %array of pose class instances - should this be property of robot or each sensor?
    end
    
    methods
    end
    
end

