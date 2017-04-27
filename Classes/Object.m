classdef Object < HeterogeneousObject
    %OBJECT class instances are used by Sensor class instances to create a
    %representation of objects in the environment.
    %   Different sensor subclasses perceive environment through
    %   specialised methods. 
    %   Objects are created and used to generate measurements.
    %	Though sensors behave differently, representing all objects with
    %	this class and subclasses will ensure uniformity
    
    %% 1. Properties
    properties%(GetAccess = 'protected', SetAccess = 'protected')
        parameters
    end
    
    
    %% 2. Methods
    % Constructor
    methods(Access = public) %set to private later??
        function self = Object()
        end
        
    end
    
    %
    methods(Access = private)
        function out = getSwitch(self,property,varargin)
            %output depends on varargin
            switch property
                case 'index'
                    out = self.index;
                case 'trajectory'
                    out = self.trajectory;
                case 'parameters'
                    out = self.parameters;
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            %depends on varargin
            switch property
                case 'index'
                    self.index = value;
                case 'parameters'
                    self.parameters = value;
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

    
        %setter for Object class arrays
%         function self = setArr(self,locations,property,values,varargin)
%             % inputs
%             %   locations is 1xn array of location indexes
%             %   property is string
%             %   values is cell array(1xn), object array(1xn) or array(mxn)
%             % outputs
%             %   self
%             
%             %loop over locations
%             for i = 1:numel(locations)
%                 %check that values is mxn array or 1xn cell array 
%                 if iscell(values)
%                     assert(isequal(size(values),[1,numel(locations)]),...
%                            'Error: Number of locations and values must be equal');
%                 	self(locations(i)) = self(locations(i)).set(property,values{i},varargin);
%                 else
%                     assert(size(values,2)==numel(locations),...
%                            'Error: Number of locations and values must be equal');
%                     self(locations(i)) = self(locations(i)).set(property,values(:,i),varargin);
%                 end
%             end
%         end
%         
%         %setter for single Object class instance
%         function self = set(self,property,value,varargin)
%             if isempty(varargin)
%                 self.(property) = value;
%             else
%                 %use varargin depending on property
%                 switch property
%                     case 'index'
%                         self.index = value;
%                     case 'parameters'
%                         self.parameters = value;
%                     case 'trajectory'
%                         self.trajectory = value;
%                 end   
%                 
%             end
%             
%         end
        
    end
    
end

