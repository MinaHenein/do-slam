%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 29/06/2017
% Contributors:
%--------------------------------------------------------------------------

function self = setVisibility(self,config,environment)
%SETVISIBILITY Sets visibility for the sensor viewing points and primitives
%in the environments as they are turned into sensor objects and Points.
t      = config.t;
nSteps = numel(t);
self.pointVisibility     = zeros(self.nPoints,nSteps);
self.objectVisibility    = zeros(self.nObjects,nSteps);
self.pointObservationRelative = GP_Point.empty();

for i = 1:nSteps
    meshes = self.generateMeshes(environment,t(i));
    for j = 1:self.nPoints
        jPoint = self.get('points',j);
        [jPointVisible,jPointRelative] = self.pointVisibleOcclusion(jPoint,meshes,t(i));
        self.pointVisibility(j,i) = jPointVisible;
        self.pointObservationRelative(j,i) = jPointRelative;
    end
end
        
end

