classdef BaseObject < matlab.mixin.Heterogeneous & matlab.mixin.Copyable & handle
    %BaseObject is a hetergeneous superclass for Object and Geometry classes
    %   This class allows Object and Geometry class & subclass instances to 
    %   be stored in a heterogeneous array.
    
    %% 1. Properties
    properties%(GetAccess = 'protected', SetAccess = 'protected')
        index %integer - unique for instances of subclasses of BaseObject
        connectedObjectIndexes %array of indexes
    end
    
    
    %% 2. Methods for Environment Class Arrays
    %matlab.mixin.Heterogeneous requires sealed methods
    methods(Access = public, Sealed = true)
        
        % Getter
        function outArr = getHet(self,locations,property,varargin)
            % inputs
            %   locations is 1xn array of location indexes, or str ':'
            %   property is string
            % outputs
            %   outArr is cell array(1xn), object array(1xn) or array(mxn)
            
            %get property at each location, store in cell array
            if strcmp(locations,':')
                locations = 1:numel(self);
            end
            outArr = cell(1,numel(locations));
            for i = 1:numel(locations)
                if isempty(varargin) == 0
                    outArr{i} = [self(locations(i)).(property)]; 
                else
                    %do something with varargin depending on property
                    switch property
                        otherwise
                            outArr{i} = [self(locations(i)).(property)];
                    end
                end
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
        
        % Setter
        function self = setHet(self,locations,property,values,varargin)
            % inputs
            %   locations is 1xn array of location indexes, or str ':'
            %   property is string
            %   values is cell array(1xn), object array(1xn) or array(mxn)
            % outputs
            %   self
            
            %loop over locations
            if strcmp(locations,':')
                locations = 1:numel(self);
            end
            for i = 1:numel(locations)
                %check that values is mxn array, 1xn object array or 1xn 
                %cell array. check correct sizes and convert array to cell
                %array
                if iscell(values)
                    assert(isequal(size(values),[1,numel(locations)]),...
                           'Error: Number of locations and values must be equal')
                else
                    assert(size(values,2)==numel(locations),...
                           'Error: Number of locations and values must be equal')
                    %convert array to cell array of columns vectors
                    values = mat2cell(values,size(values,1),ones(1,numel(locations)));
                end
                %set properties
                if isempty(varargin)
                    self(locations(i)).(property) = values{i};
                else
                    %do something with varargin depending on property
                    switch property
                        otherwise
                            self(locations(i)).(property) = values{i};
                    end
                end
            end
            
        end
        
    end
    
end

