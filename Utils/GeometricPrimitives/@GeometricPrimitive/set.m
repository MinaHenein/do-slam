function obj = set(obj,property,varargin)
%SET used to set properties of the Geometric Primitive.

switch property
    case 'type'
        switch varargin{1}
            case 'box'
                obj = GPBox();
            case 'cylinder'
                obj = GPCylinder();
            case 'capsule'
                obj = GPCapsule();
            case 'sphere'
                obj = GPSphere();
            case 'surface' % rectangle
                obj = GPSurface();
            case 'plane'
                obj= GPPlane();
            otherwise
                error('Not implemented type input.')
        end
    case 'params'
        [~,isParamsDimensionCorrect] = ...
            checkDimension(obj.type,varargin{1});
        if isParamsDimensionCorrect
            obj.params = varargin{1};
        else
            error('Incorrect type/parameters dimension.')
        end
        
    case 'pose'
        [isPoseDimensionCorrect,~] = ...
            checkDimension(obj.type,varargin{1});
        if isPoseDimensionCorrect
            obj.pose = varargin{1};
        else
            error('Incorrect type/pose dimension.')
        end
        
    otherwise
        error('Incorrect property input.')
end

end

function [isPoseDimensionCorrect, isParamsDimensionCorrect] = checkDimension(type,pose,params)

isPoseDimensionCorrect = 0;
isParamsDimensionCorrect = 0;

if ~isempty(type)
    switch type
        case 'box'
            if length(pose) == 6
                isPoseDimensionCorrect = 1;
            end
            if length(params) == 3 % length, width, height
                isParamsDimensionCorrect = 1;
            end
        case 'cylinder'
            if length(pose) == 6
                isPoseDimensionCorrect = 1;
            end
            if length(params) == 2 % radius, length
                isParamsDimensionCorrect = 1;
            end
        case 'capsule' % capped cylinder
            if length(pose) == 6
                isPoseDimensionCorrect = 1;
            end
            if length(params) == 2 % radius, length
                isParamsDimensionCorrect = 1;
            end
        case 'sphere'
            if length(pose) == 3
                isPoseDimensionCorrect = 1;
            end
            if length(params) == 1 % radius
                isParamsDimensionCorrect = 1;
            end
        case 'surface' % rectangle
            if length(pose) == 6
                isPoseDimensionCorrect = 1;
            end
            if length(params) == 2 % length, width
                isParamsDimensionCorrect = 1;
            end
        case 'plane'
            if length(params) == 4 % eta, distance
                isParamsDimensionCorrect = 1;
            end
            disp('A plane type object has no pose.')
        otherwise
            error('Not implemented type input.')
    end
else
    error('Can not set pose of a geometric primitive that has no type assigned.')
end


end