function varargout = get(obj, field, varargin)
%GET function for the Environment Primitive Class.

format = 'Log(SE(3))'; %default format is this

    switch field
        case 'ID'
            varargout{1} = obj.ID;
        case 'AbsolutePose'
            % obtains the pose of the object in the desired format
            time = varargin{1};
            if numel(varargin)==2
                format = varargin{2}; % if format is specified, use it
            end
            pose = obj.Trajectory.get('Pose',time,format); % update this line based on trajectory
            varargout{1} = pose;

        case 'RelativePose'
            frame = varargin{1};
            time = varargin{2};
            AbsolutePose = obj.get('Pose',time,format);
            if numel(varargin)==3
                format = varargin{3}; % if format is specified, use it
                % convert to SE(3)
            end
            RelativePose = Absolute2RelativePose(AbsolutePose,frame);
            % convert to different format if required
            varargout{1} = RelativePose;

        case 'AbsolutePoints'
            time = varargin{1};
            pose = obj.get('AbsolutePose',time); % set default to Log(SE(3))
            points = Relative2AbsolutePoints3D(pose,obj.points);
            varargout{1} = points;

        case 'RelativePoints';
            frame = varargin{1};
            time  = varargin{2}; % time is the third argument input
            pose = obj.get('AbsolutePoints',time); % obtains the pose
            points = Relative2AbsolutePoints3D(pose,obj.points);
            % ADD CODE HERE to convert format if specified
            RelativePoints = AbsolutePoints2RelativePoints3D(frame,points);
            varargout{1} = RelativePoints;

        case 'AbsoluteMeshTriangles'
            time = varargin{1};
            MeshTriangles = obj.MeshTriangles;
            pose = obj.get('Pose',time);
            % transform mesh point vertices with object pose
            MeshTriangles(:,1:3) = Relative2AbsolutePoints3D(pose,MeshTriangles(:,1:3));
            MeshTriangles(:,4:6) = Relative2AbsolutePoints3D(pose,MeshTriangles(:,4:6));
            MeshTriangles(:,7:9) = Relative2AbsolutePoints3D(pose,MeshTriangles(:,7:9));
            varargout{1} = MeshTriangles;

        case 'RelativeMeshTriangles'
            time = varargin{1};
            MeshTriangles = obj.get('AbsoluteMeshTriangles',time);
            % create relative observation for the mesh
            MeshTriangles(:,1:3) = AbsolutePoints2RelativePoints3D(frame,MeshTriangles(:,1:3));
            MeshTriangles(:,4:6) = AbsolutePoints2RelativePoints3D(frame,MeshTriangles(:,4:6));
            MeshTriangles(:,7:9) = AbsolutePoints2RelativePoints3D(frame,MeshTriangles(:,7:9));
            varargout{1} = MeshTriangles;

        case 'RelativePointsOccluded'
            frame = varargin{1};
            time = varargin{2};
            RelativePoints = obj.get('RelativePoints',frame,time);
            RelativeMeshTriangles = obj.get('RelativeMeshTriangles',frame,time);
            % NEED SOME WAY OF ENSURING DATA ASSOCIATION between points
            [varargout{1}, varargout{2}] = OccludePoints(RelativePoints,RelativeMeshTriangles);
            varargout{3} = RelativeMeshTriangles;

        case 'nPoints'
            varargout{1} = size(obj.Points,1);

        otherwise
            error('Incorrect field type input.')
    end

end

