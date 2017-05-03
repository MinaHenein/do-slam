classdef EP_Rectangle < EnvironmentPrimitiveALT
    %EP_RECTANGLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sideLengths
    end
    
    methods(Access = public)
        % Constructor
        function self = EP_Rectangle(sideLengths,trajectory)
            self.sideLengths = sideLengths;
            self.trajectory = trajectory;
        end
        
        %plot
    end
    
    
end

