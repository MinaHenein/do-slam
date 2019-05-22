function plotVelocityProfiles(objectPosesCell, fps, sequenceName)

nObjects = size(objectPosesCell,2);
objectsMotion = {};

% calculate object motion in camera frame
for i=1:nObjects
    objectPoses = objectPosesCell{1,i};
    objectMotion = zeros(6,(size(objectPoses,2))-1);
    for k=2:size(objectPoses,2)
        objectMotion(:,k-1) = transformationMatrixToPose(poseToTransformationMatrix(objectPoses(:,k))/...
            poseToTransformationMatrix(objectPoses(:,k-1)));
    end
    objectsMotion{i} = objectMotion;
end

% get linear velocity and plot
for i=1:nObjects
    objectMotions = objectsMotion{i};
    objectLinearVelocity = zeros(1,size(objectMotions,2));
    for j=1:size(objectMotions,2)
        vw = LogSE3(poseToTransformationMatrix(objectMotions(:,j)));
        v = vw(1:3);
        % w = vw(4:6);        
        objectLinearVelocity(j) = norm(v);
    end
    objectsLinearVelocities{i} = objectLinearVelocity;
end

fig = figure;   
for i=1:nObjects
    subplot(ceil(nObjects/2),2,i);
    plot(1:length(objectsLinearVelocities{i}),fps*3.6*objectsLinearVelocities{i})
end
print(fig,'-dpdf',strcat(num2str(sequenceName),'_0.pdf'),'-bestfit');

end