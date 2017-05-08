classdef EP_Default < EnvironmentPrimitiveALT
    %EP_DEFAULT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    %% 2. Methods
    % Getter & Setter
%     methods(Access = public) %set to protected later??
%         function out = get(self,property)
%         	out = [self.(property)];
%         end
%         
%         function self = set(self,property,value)
%         	self.(property) = value;
%         end
%     end
    
    
    methods(Access = public)
        function self = EP_Default()
        end
    end
    
end

