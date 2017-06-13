function varargout = get(obj, property, varargin)
%GET function for the GPBox Class.

    switch property
        case 'length'
            varargout{1} = obj.length;
        case 'width'
            varargout{1} = obj.width;
        case 'height'
            varargout{1} = obj.height;
        otherwise
            error('Incorrect property input.')
    end

end