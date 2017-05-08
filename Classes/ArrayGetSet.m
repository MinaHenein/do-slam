classdef ArrayGetSet < handle & matlab.mixin.Copyable
    %ARRAYGETSET implements get and set for object arrays
    %   All classes required get/set will inherit from ArrayGetSet
    %   All subclasses must implement their own getSwitch & setSwitch
    %   methods
    
    %% 1. Properties
    properties
    end
    
    %% 2. Methods
    methods(Access = public, Sealed = true)
        % Get
        %   assumes self is 1xn object array
        %   if location provided, gets property of each self(location)
        %   if no location provided, gets property for whole array
        %   property values from each location put in cell array, converted
        %   to object array or array if possible
        %   uses getSwitch - a method of the class of self(location(i))
        function values = get(self,varargin)
            assert(any(size(self)==1),'Object arrays must be row or column vector')
            if ischar(varargin{1})
                % first input is property string - get all
                locations  = 1:numel(self);
                valuesCell = cell(size(self));
                property   = varargin{1};
                vararginStart = 2;
            else
                %first input is array of locations
                locations  = varargin{1};
                property   = varargin{2};
                valuesCell = cell(size(self(locations)));
                vararginStart = 3;
            end
            %store required values in cell array
            for i = 1:numel(locations)
                valuesCell{i} = self(locations(i)).getSwitch(property,varargin{vararginStart:end});
            end
            %convert valuesCell to array/object array
            try
                values = cell2mat(valuesCell);
            catch
                try
                    cellSizes = cellfun('length',valuesCell);
                    cellIndexes = [0 cumsum(cellSizes)];
                    values(sum(cellSizes)) = feval(class(valuesCell{1}));
                    for i = 1:numel(valuesCell)
                        values(cellIndexes(i)+1:cellIndexes(i+1)) = valuesCell{i};
                    end
                catch
                    values = valuesCell;    
                end
            end
        end
        
        % Set
        %   assumes self is 1xn object array
        %   if location provided, sets property of each self(location)
        %   if no location provided, sets property for whole array
        %   values for each location must be provided as cell array, object
        %   array or array
        %   no. values provided must match locations:
        %       -if cell array - 1:1
        %       -if object array/array, columns must be divisible by no.
        %       locations
        %   uses setSwitch - a method of the class of self(location(i))
        function self = set(self,varargin)
            if ischar(varargin{1})
                %first input is property string - set all
                locations = 1:numel(self);
                property  = varargin{1};
                values    = varargin{2};
                vararginStart = 3;
            else
                %first input is array of locations
                locations = varargin{1};
                property  = varargin{2};
                values    = varargin{3};
                vararginStart = 4;
            end
            %convert values to cell array with same no. elements as
            %locations
            if iscell(values)
                assert(numel(values)==numel(locations),'Error: Number of locations and values must match')
                valuesCell = values;
            else
                nColumns = size(values,2);
                width = nColumns/numel(locations);
                assert(mod(nColumns,numel(locations))==0,'Error: Number of columns must be equally distributable among locations')
                valuesCell = mat2cell(values,size(values,1),repmat(size(values,2),nColumns/width));
            end
            %setSwitch
            for i = 1:numel(locations)
                self(locations(i)).setSwitch(property,valuesCell{i},varargin{vararginStart:end});
            end
        end
    end
    
end

