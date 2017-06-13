function varargout = get(obj, property, varargin)
%GET function for the Geometric Primitive Class.

    switch property
        case 'type'
            varargout{1} = obj.type;
        case 'params'
            varargout{1} = obj.params;
        case 'pose'
            varargout{1} = obj.pose;
        otherwise
            error('Incorrect property input.')
    end

end

