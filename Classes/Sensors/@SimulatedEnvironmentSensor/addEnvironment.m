%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 02/06/18
%--------------------------------------------------------------------------
function self = addEnvironment(self,environment)
    points(environment.nEnvironmentPoints) = Point();
    %loop over environmentPoints, create Points
    for i = 1:environment.nEnvironmentPoints
        points(i) = Point(environment.get('environmentPoints',i));
    end

    %loop over environmentPrimitives, create objects
    objects(environment.nEnvironmentPrimitives) = SensorObject();
    for i = 1:environment.nEnvironmentPrimitives
        switch class(environment.get('environmentPrimitives',i))
            case 'EP_Rectangle'
                objects(i) = GEO_Plane(environment.get('environmentPrimitives',i));
            case 'EP_default'
                objects(i) = self.RBOfromEP(environment.get('environmentPrimitives',i)); % runs special internal function for rigid body
                % that creates from Environment Primitives
            otherwise
                error('Error: object conversion for %s not yet implemented',class(environment.get('environmentPrimitives',i)))
        end
    end

    %add properties
    self.points  = points;
    self.objects = objects;
end
