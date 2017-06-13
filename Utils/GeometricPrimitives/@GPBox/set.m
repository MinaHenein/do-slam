function obj = set(obj,property,varargin)
%SET used to set properties of the GPBox.

switch property
    case 'length'
        if ~isempty(obj.params) && obj.params(1)==obj.params(2) && obj.params(2)==obj.params(3)
            disp('WARNING! Changing dimensions of a cube, shape will not be preserved')
        end
        obj.length = varargin{1};
        if ~isempty(obj.params)
        obj.params = [obj.length, obj.width, obj.height];
        end
    case 'width'
        if ~isempty(obj.params) && obj.params(1)==obj.params(2) && obj.params(2)==obj.params(3)
            disp('WARNING! Changing dimensions of a cube, shape will not be preserved')
        end
        obj.width = varargin{1};
        if ~isempty(obj.params)
        obj.params = [obj.length, obj.width, obj.height];
        end
    case 'height'
        if ~isempty(obj.params) && obj.params(1)==obj.params(2) && obj.params(2)==obj.params(3)
            disp('WARNING! Changing dimensions of a cube, shape will not be preserved')
        end 
        obj.height = varargin{1};
        if ~isempty(obj.params)
        obj.params = [obj.length, obj.width, obj.height];
        end
    otherwise
        error('Incorrect property input.')
end

end
