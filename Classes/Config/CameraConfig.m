%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef CameraConfig < Config
    %CAMERACONFIG is example of Config for specific application
    %   This subclass used to add properties specific to a certain
    %   application that should not be in base Config class
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'private')
        focalLength
        opticalCentreX
        opticalCentreY
        fieldOfView
        cameraRelativePose
    end
    
    properties (Dependent)
        intrinsics
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public) 
        function self = CameraConfig()
            self.initPath();
        end
    end
    
    % Get & Set
    methods(Access = public) 
    	function out = getSwitch(self,property)
        	out = self.(property);
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Dependent properties
    methods
        function intrinsics = get.intrinsics(obj)
            intrinsics = [obj.focalLength;obj.opticalCentreX;obj.opticalCentreY];
        end
    end
end