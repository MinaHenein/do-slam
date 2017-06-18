%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 31/05/17
%--------------------------------------------------------------------------

classdef EnvironmentPrimitive < ArrayGetSet & matlab.mixin.Heterogeneous
    %ENVIRONMENTPRIMITIVE represents a primitive in the environment
    %   This is a base class from which specific primitives inherit
    %   It inherits from matlab.mixin.Heterogeneous which allows different
    %   environment primitives to be stored together in object arrays
    
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

