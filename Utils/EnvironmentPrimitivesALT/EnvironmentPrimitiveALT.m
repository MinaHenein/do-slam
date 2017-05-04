classdef EnvironmentPrimitiveALT < handle & matlab.mixin.Copyable & matlab.mixin.Heterogeneous
    %ENVIRONMENTPRIMITIVEALT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        index
        trajectory
        environmentPointIndexes
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public, Sealed) %if subclasses need special gets/sets - remove these
        function out = get(self,property)
        	out = [self.(property)];
        end
        
        function self = set(self,property,value)
        	self.(property) = value;
        end
    end
    
    
%     methods(Access = public)
%         function self = EnvironmentPrimitiveALT()
%         end
%     end
    
end

