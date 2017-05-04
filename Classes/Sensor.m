classdef Sensor < matlab.mixin.Copyable & handle
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = get(self,property)
        	out = [self.(property)];
        end
        
        function self = set(self,property,value)
        	self.(property) = value;
        end
    end
    
end

