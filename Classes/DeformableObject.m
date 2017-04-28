classdef DeformableObject < BaseObject
    %DEFORMABLEOBJECT class instances are used by Sensor class instances to 
    %create a representation of rigid body objects in the environment.
    %   Deformable objects have trajectory (ie a person) - will likely be
    %   composed of other attached objects with their own trajectories
    %   relative to the trajectory of the deformable object
    
     %% 1. Properties
    properties
        trajectory
    end
    
    %% 2. Methods
    methods(Access = private)
        function out = getSwitch(self,property,varargin)
            %output depends on varargin
            switch property
                case 'index'
                    out = self.index;
                case 'trajectory'
                    out = self.trajectory;
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            %depends on varargin
            switch property
                case 'index'
                    self.index = value;
                case 'trajectory'
                    self.trajectory = value;
            end
        end
    end
    
    % Getters & Setters
    methods(Access = public) %set to protected later??
        function out = get(self,varargin)
            if ischar(varargin{1}) && ~strcmp(':',varargin{1})
                %no location given - self is single Object class instance
                property = varargin{1};
                if (nargin == 2) %just output property
                    assert(size(self,2)==1,...
                           'Pass ":" as location for Object class arrays if all values desired')
                    out = self.(property);
                else
                    out = self.getSwitch(property,varargin{2:end});
                end
            else
                %location given - self is Object class array
                if ischar(varargin{1}) && strcmp(':',varargin{1})
                    locations = 1:size(self,2);
                else
                    locations = varargin{1};
                end
                property = varargin{2};
                outArr   = cell(1,numel(locations));
                for i = 1:numel(locations)
                    if (nargin==3)
                        outArr{i} = self(locations(i)).(property);
                    else
                        outArr{i} = self(locations(i)).getSwitch(property,varargin{3:end});
                    end
                end
                %convert to array or object array if possible, otherwise
                %return cell array
                try
                    out = cell2mat(outArr);
                catch
                    try
                        out = feval(class(outArr{1}));
                        for i = 1:numel(locations)
                            out(i) = outArr{i};
                        end
                    end
                end
            end
        end
        
        function self = set(self,varargin)
            if ischar(varargin{1}) && ~strcmp(':',varargin{1})
                %no location given - self is single Object class instance
                property = varargin{1};
                values   = varargin{2};
                if (nargin == 3) %just output property
                    assert(size(self,2)==1,...
                           'Pass ":" as location for Object class arrays if all values desired')
                    self.(property) = values;
                else
                    self.setSwitch(property,values,varargin{3:end});
                end
            else
                %location given - self is Object class array
                if ischar(varargin{1}) && strcmp(':',varargin{1})
                    locations = 1:size(self,2);
                else
                    locations = varargin{1};
                end
                property = varargin{2};
                values   = varargin{3};
                if iscell(values)
                    assert(isequal(size(values),[1,numel(locations)]),...
                           'Error: Number of locations and values must be equal')
                else
                    assert(size(values,2)==numel(locations),...
                           'Error: Number of locations and values must be equal')
                    %convert array to cell array of columns vectors
                    values = mat2cell(values,size(values,1),ones(1,numel(locations)));
                end
                for i = 1:numel(locations)
                    if (nargin==4)
                        self(locations(i)).(property) = values{i};
                    else
                        self(locations(i)).setSwitch(property,values{i},varargin{4:end});
                    end
                end
            end
        end
        
    end
    
end

