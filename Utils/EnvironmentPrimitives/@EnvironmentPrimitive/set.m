function obj = set(obj,field,varargin)
    %SET used to set properties of the Enrivonment Primitive.

    switch field
        case 'Points'
            if size(varargin{1},2)==3
                obj.Points = varargin{1};
            else
                error('Incorrect Point dimensions.')
            end

        case 'Trajectory'
            if isa(varargin{1},'Trajectory')
                obj.Trajectory = varargin{1};
            else
                error('Input is not of Trajectory type.')
            end

        case 'MeshTriangles'
            if size(varargin{1},2)==9
                obj.MeshTriangles = varargin{1};
            else
                error('Input format for MeshTriangles is incorrect.')
            end

        otherwise
            error('Field type incorrect.')
    end

end

