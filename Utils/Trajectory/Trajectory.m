classdef Trajectory < matlab.mixin.Copyable & handle
    %TRAJECTORY class represents trajectories of moving/stationary
    %components (robots, objects, points )
    %   Both dynamic and static objects will use trajectory class. Only
    %   difference is model used.
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'public')
    end
    properties(GetAccess = 'protected', SetAccess = 'protected')
        t          %1xn array of time values
        model      %function handle of trajectory model
    end
    
    
    %% 2. Methods       
    % PoseTrajectory and PositionTrajectory have their own get and set,
    % otherwise these functions would not be able to access the 'poses' or
    % 'positions' properties which are only properties of the respective
    % subclasses
end

