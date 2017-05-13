classdef Robot < matlab.mixin.Copyable & handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory %trajectory class instance
        sensors    %array of sensor class instances
    end
    
    %% 2. Methods
    methods
    end
    
end

