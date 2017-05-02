clear all
% close all

%% Getting and setting
%   1. Each subclass of BaseObject class has its own get and set. 
%   2. 'get' requires property(string). Location can be provided as an
%      additional first input.
%   3. getting from multiple locations: output is either array, object
%      array or cell array, depending on value of property. If location was
%      (1xn) array, output will be (mxn) array, (1xn) object array or (1xn)
%      cell array
%   4. 'set' requires property(string) and value. Location can be provided
%      as an additional first input. 
%   5. For multiple locations, multiple values are required. They must be
%      stacked in array, object array or cell array depending on value of
%      property. If location is (1xn) array, value msut be (mxn) array, 
%      (1xn) object array or (1xn) cell array
%   6. The string ':' can be passed as the location input, meaning all
%      elements of the array.
%   7. BaseObject does not have a constructor - it will only be created when
%      mixed types are put in one array. Thus, its 'get' and 'set' require
%      location to be given explicitly.
%   8. Since BaseObject is a direct subclass of matlab.mixin.Heterogeneous,
%      its methods must be sealed and cannot be redefined in subclasses.
%      The suffix 'Het' is added to avoid naming clashes with subclasses.

%trajectories (just placeholders - see TrajectoryInitialisation application
%              for trajectory examples)
t1 = Trajectory();
t2 = Trajectory();

%create geometric object
geoObj = GeometricObject();
geoObj.set('index',1);
geoObj.set('trajectory',t1);
geoObj.set('parameters',pi);

%create rigid body object
rbObj = RigidBodyObject();
rbObj.set('index',2);
rbObj.set('trajectory',t1);

%create point
pt = Point();
pt.set('index',3);
pt.set('trajectory',t2);

%geometric object array
geoObjArr = [geoObj.copy() geoObj.copy()]; %pass by reference, need to copy like this
geoObjArr.get(':','index')
geoObjArr.set(1,'index',10);
geoObjArr.get([1 2],'index')
geoObjArr.get(':','trajectory')

%base array
baseObjArr = [geoObj.copy() rbObj.copy() pt.copy()]; %pass by reference, need to copy like this

%get and set environment array
baseObjArr.getHet([1 2 3],'index')
baseObjArr.setHet([2 1],'index',[200 100]);
baseObjArr.getHet(':','index')
