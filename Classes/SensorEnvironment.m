classdef SensorEnvironment
    %SENSORENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        objects
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = SensorEnvironment(environment)
            self.objects = BaseObject();
            self.objects(environment.nEnvironmentPoints + environment.nEnvironmentPrimitives) = BaseObject();
            
            objectCount = 0;
            %create points from environmentPoints
            for i = 1:environment.nEnvironmentPoints
                objectCount = objectCount + 1;
                self.objects(objectCount) = Point();
                self.objects(objectCount).set('index',objectCount);
            end
            
            %create objects from environmentPrimitives
            for i = 1:environment.nEnvironmentPrimitives
                objectCount = objectCount + 1;
                switch class(environment.get('environmentPrimitives',i))
                    case  'EP_Rectangle'
                        self.objects(objectCount) = GeometricEntity();
                    otherwise
                        error('Error: %s conversion not implemented',class(environment.get('environmentPrimitives',i)))
                end
                self.objects(objectCount).set('index',objectCount);
            end
        end
    end
    
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

