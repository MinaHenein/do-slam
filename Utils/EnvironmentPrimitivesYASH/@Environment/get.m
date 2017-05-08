function varargout = get(obj,field,varargin)
%GET All the get functions.

format = 'Log(SE(3))'; % default format

switch field
    case 'EnvironmentPrimitives'
        if numel(varargin)==0
            varargout = obj.EnvironmentPrimitives;
        elseif numel(varargin)==1
            varargout = obj.EnvironmentPrimitives(varargin{1});
        end
        
    case 'nEnvironmentPrimitives'
        varargout = numel(obj.EnvironmentPrimitives);
        
    case 'EnvironmentPrimitiveAbsolutePoses'
        indexes = 1:1:numel(obj.EnvironmentPrimitives);
        if ~isempty(varargin{1})
            indexes = varargin{1};
        end
        time = varargin{2};
        if numel(varargin)>2
            format = varargin{3};
        end
        varargout = obj.EnvironmentPrimitives(indexes).get('AbsolutePose',time,format);
        
    case 'EnvironmentPrimitiveRelativePoses'
        indexes = 1:1:numel(obj.EnvironmentPrimitives);
        if ~isempty(varargin{1})
            indexes = varargin{1};
        end
        time = varargin{2};
        frame = varargin{3};
        if numel(varargin)>3
            format = varargin{4};
        end
        varargout = obj.EnvironmentPrimitives(indexes).get('RelativePose',time,frame,format);
        
    otherwise
        error('Incorrect field type.')
        

end

