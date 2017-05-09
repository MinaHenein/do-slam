classdef EnvironmentPrimitive < ArrayGetSet & matlab.mixin.Heterogeneous
    %ENVIRONMENTPRIMITIVEALT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        index
        trajectory
        pointIndexes %points for which this primitive is referencePrimitive
    end
    
    %% 2. Methods
    % Get & Set
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
        	out = self.(property);
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
        
    end
end

