classdef Point < BaseObject
    %POINT class instances are used by Sensor class instances to create a
    %representation of points in the environment.
    %   Points are created and used to generate measurements
    
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        trajectory
    end
    
    
    %% 2. Methods
    % Constructor
    methods(Access = public) %set to private later??
        function self = Point()
        end
        
    end
    
    % Getters & Setters
    methods(Access = public) %set to protected later??
        %getter for Object class arrays
        function outArr = getArr(self,locations,property,varargin)
            % inputs
            %   locations is 1xn array of location indexes
            %   property is string
            % outputs
            %   outArr is cell array(1xn), object array(1xn) or array(mxn)
            
            %get property at each location, store in cell array
            outArr = cell(1,numel(locations));
            for i = 1:numel(locations)
                outArr{i} = self(locations(i)).get(property,varargin);    
            end
            %convert to array or object array if possible, otherwise return
            %cell array
            try
                outArr = cell2mat(outArr);
            catch
                try
                    outCell = outArr;
                    outArr = feval(class(outArr{1}));
                    for i = 1:numel(locations)
                        outArr(i) = outCell{i};
                    end
                end
            end
        end

        %getter for single Object class instance
        function out = get(self,property,varargin)
            if isempty(varargin)
                out = [self.(property)]; 
            else
                %do something with varargin
                switch property
                    case 'index'
                        out = [self.index];
                    case 'trajectory'
                        out = [self.trajectory];
                end
            end
            
        end
    
        %setter for Object class arrays
        function self = setArr(self,locations,property,values,varargin)
            % inputs
            %   locations is 1xn array of location indexes
            %   property is string
            %   values is cell array(1xn), object array(1xn) or array(mxn)
            % outputs
            %   self
            
            %loop over locations
            for i = 1:numel(locations)
                %check that values is mxn array or 1xn cell array 
                if iscell(values)
                    assert(isequal(size(values),[1,numel(locations)]),...
                           'Error: Number of locations and values must be equal');
                	self(locations(i)) = self(locations(i)).set(property,values{i},varargin);
                else
                    assert(size(values,2)==numel(locations),...
                           'Error: Number of locations and values must be equal');
                    self(locations(i)) = self(locations(i)).set(property,values(:,i),varargin);
                end
            end
        end
        
        %setter for single Object class instance
        function self = set(self,property,value,varargin)
            if isempty(varargin)
                self.(property) = value;
            else
                %use varargin depending on property
                switch property
                    case 'index'
                        self.index = value;
                    case 'trajectory'
                        self.trajectory = value;
                end   
                
            end
            
        end
        
    end
    
end

