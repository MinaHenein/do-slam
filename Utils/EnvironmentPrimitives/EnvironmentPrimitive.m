classdef EnvironmentPrimitive < handle
    %GEOMETRICPRIMITIVE Contains a set of Points and
    %Trajectory, used to create sensor observation objects.
    % 
    
    properties (Access=private)
        Points
        MeshTriangles
        Trajectory
    end
    
    properties (Dependent)
        nPoints
        nTimeSteps
        startTime
        endTime
    end
    
    methods
%         function obj = GeometricPrimitive(varargin)
%             Constructor, default
%             if varargin{1} && size(varargin{1},2)==3
%                 obj.Points = varargin{1};
%             end
%             if varargin{2} && isa(varargin{2},'Trajectory')
%                 obj.Trajectory = varargin{2};
%             end
%             if varargin{3} && isa(varargin{3},'MeshTriangles')
%                 obj.MeshTriangles = varargin{3};
%             end            
%         end
        
        function pose = getPose(obj,time,format)
            % obtains the pose of the object in the desired format
            pose = obj.Trajectory.getPose(time,format);
        end
        
        function obj = addPoints(obj,Points)
            obj.Points = Points;
        end
        
        function obj = setTrajectory(obj, Trajectory)
            % Sets object trajectory with the given points
            if isa(Trajectory,'Trajectory')
                obj.trajectory = Trajectory;
            else
                error('Input must be of Trajectory type')
            end
        end
        
        function PointReadings = getPointReadings(obj,frame,time)
            % Returns the point Readings in the relative frame you
            % provide
            pose = obj.getPose(time,'Log(SE(3))'); % obtains the pose
            points = Relative2AbsolutePoints3D(pose,obj.points);
            PointReadings = AbsolutePoints2RelativePoints3D(frame,points);
        end
        
        function MeshTriangles = getMeshTriangles(obj,frame,time)
            % Get the triangles for the Mesh, observed in a given frame, at
            % a time step.
            MeshTriangles = obj.MeshTriangles;
            pose = obj.getPose(time);
            % transform mesh point vertices with object pose
            MeshTriangles(:,1:3) = Relative2AbsolutePoints3D(pose,MeshTriangles(:,1:3));
            MeshTriangles(:,4:6) = Relative2AbsolutePoints3D(pose,MeshTriangles(:,4:6));
            MeshTriangles(:,7:9) = Relative2AbsolutePoints3D(pose,MeshTriangles(:,7:9));
            
            % create relative observation for the mesh
            MeshTriangles(:,1:3) = AbsolutePoints2RelativePoints3D(frame,MeshTriangles(:,1:3));
            MeshTriangles(:,4:6) = AbsolutePoints2RelativePoints3D(frame,MeshTriangles(:,4:6));
            MeshTriangles(:,7:9) = AbsolutePoints2RelativePoints3D(frame,MeshTriangles(:,7:9));
        end
        
        function [PointReadingsOccluded, meshTriangles] = getPointReadingsOccluded(obj,frame,time)
            % Obtain PointReadings occluded by the internal appearance
            % model of the GP. Must input a frame to obtain the
            % PointReadings in. Can also output the meshTriangles for
            % inter-Object occlusion.
            PointReadings = getPointReadings(obj,frame,time);
            meshTriangles = obj.getMeshTriangles(frame,time);
            PointReadingsOccluded = occludePointReadings(PointReadings,meshTriangles);
        end
        
        function nPoints = getnPoints(obj)
            nPoints = size(obj.Points,1);
        end
        
        function nTimeSteps = getTimeSteps(obj)
            nTimeSteps = obj.Trajectory.getTimeLength();
        end
        
        function startTime = getStartTime(obj)
            startTime = obj.Trajectory.getStart();
        end
        
        function endTime = getEndTime(obj)
            endTime = obj.Trajectory.getEnd();
        end
    end
    
end

