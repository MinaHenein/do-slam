clear all
close all

addpath(genpath(pwd))
%%

%trajectories
t1 = Trajectory('R3xSO3','continuous',@ConstantLinearVelocity);
t2 = Trajectory('R3','discrete',0:0.1:1,rand(3,10));

%create object
obj = Object();
obj.set('index',1);
obj.set('trajectory',t1);
obj.set('parameters',pi);

%create point
pt = Point();
pt.set('index',2);
pt.set('trajectory',t2);

%object array
objArr = [obj.copy() obj.copy()]; %pass by reference, need to copy like this
objArr.get(':','index')
objArr.set(1,'index',10);
objArr.get([1 2],'index')
objArr.get(':','trajectory')


%store in heterogeneous array
hetArr = [obj.copy() pt.copy()]; %pass by reference, need to copy like this

%get and set environment array
hetArr.getHetArr([1 2],'index')
hetArr.setHetArr([2 1],'index',[200 100]);
hetArr.getHetArr(':','index')
hetArr.getHetArr([1 2],'trajectory')
