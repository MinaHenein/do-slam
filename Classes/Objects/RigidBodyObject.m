%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 30/05/17
%--------------------------------------------------------------------------

classdef RigidBodyObject < SensorObject
    %RIGIDBODYOBJECT class instances are used by Sensor class instances to 
    %create a representation of rigid body objects in the environment.
    %   Rigid body objects have trajectory (ie a car) - will likely be
    %   composed of other attached objects fixed to the rigid body
    
     %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
        mesh
    end
    
    %% 2. Methods
    methods
        function self = RBOfromEP(self, EP_default)
            % creates Rigid Body Object from EP
            mesh = EP_default.get('mesh');
            if size(mesh,2)==9
                self.set('mesh', mesh);
            else
                error('Mesh input incorrect.')
            end
            self.set('trajectory', EP_default.get('trajectory'));
            self.set('index', EP_default.get('index'));
            self.set('pointIndexes', EP_default.get('pointIndexes'));
        end
    
    end
end

