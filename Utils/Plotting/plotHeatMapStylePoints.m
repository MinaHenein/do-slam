function plotHeatMapStylePoints(pointsGT,pointsN,NoSE3_pointsN,...
    posePoints,posePointsNoSE3,posesGT,posesN, NoSE3_posesN,relPose,relPoseNoSE3)

n = size(pointsN,2);
SE3_translation_error = zeros(n,1);
NoSE3_translation_error = zeros(n,1);

for i=1:n
    pointsN(:,i) = RelativeToAbsolutePositionR3xso3(transformationMatrixToPose(posePoints),pointsN(:,i));
    NoSE3_pointsN(:,i) = RelativeToAbsolutePositionR3xso3(transformationMatrixToPose(posePointsNoSE3),...
        NoSE3_pointsN(:,i));
end


for i = 1:n
    % calculate the errors for solution with SE3
    v_error = pointsGT(:,i) - pointsN(:,i);
    f_trans_error2 = norm(v_error(1:3))^2;
    SE3_translation_error(i,1) = sqrt(f_trans_error2);
    % calculate the errors for solution with NoSE3
    v_error = pointsGT(:,i) - NoSE3_pointsN(:,i);
    f_trans_error2 = norm(v_error(1:3))^2;
    NoSE3_translation_error(i,1) = sqrt(f_trans_error2);
end

minSE3Distance  = min(SE3_translation_error);
minNoSE3Distance  = min(NoSE3_translation_error);
%assert(minSE3Distance < minNoSE3Distance)
maxSE3Distance  = max(SE3_translation_error);
maxNoSE3Distance  = max(NoSE3_translation_error);
%assert(maxSE3Distance < maxNoSE3Distance)

a = linspace(minSE3Distance,maxNoSE3Distance,7);

figure('units','normalized','color','w');
axis([40 165 -350 -320])
ax = gca;
ax.Box = 'on';
for i = 1:n
    switch ceil((SE3_translation_error(i)-a(1))/(a(2)-a(1)))
        case 1; color = rgb('light blue'); 
        case 2; color = rgb('blue'); 
        case 3; color = rgb('dark blue');
        case 4; color = rgb('light red');
        case 5; color = rgb('red');
        case 6; color = rgb('dark red');
    end
    scatter3(pointsN(1,i),pointsN(2,i),pointsN(3,i),15,'MarkerEdgeColor',color,'MarkerFaceColor',color);
    hold on
end
plotPoses = plot3(posesGT(1,:),posesGT(2,:),posesGT(3,:),'Color','g','Marker','.','LineStyle','none');
set(plotPoses,'MarkerSize',7);
hold on
for i = 1:size(posesN,2)
    iPose = posesN(:,i);
    iPose = RelativeToAbsolutePoseR3xso3(iPose,relPose);
    plotCoordinates(iPose(1:3,:),rot(iPose(4:6,1))) % plots the trajectory as axes
    hold on
end
xlabel('x')
ylabel('y')
xlabel('z')

figure('units','normalized','color','w');
axis([40 165 -350 -320])
ax = gca;
ax.Box = 'on';
for i = 1:n
    switch ceil((NoSE3_translation_error(i)-a(1))/(a(2)-a(1)))
        case 1; color = rgb('light blue'); 
        case 2; color = rgb('blue'); 
        case 3; color = rgb('dark blue');
        case 4; color = rgb('light red');
        case 5; color = rgb('red');
        case 6; color = rgb('dark red');
    end
    scatter3(NoSE3_pointsN(1,i),NoSE3_pointsN(2,i),NoSE3_pointsN(3,i),15,'MarkerEdgeColor',color,'MarkerFaceColor',color);
    hold on
end
plotPoses = plot3(posesGT(1,:),posesGT(2,:),posesGT(3,:),'Color','g','Marker','.','LineStyle','none');
set(plotPoses,'MarkerSize',7);
hold on
for i = 1:size(NoSE3_posesN,2)
    iPose = NoSE3_posesN(:,i);
    iPose = RelativeToAbsolutePoseR3xso3(iPose,relPoseNoSE3);
    plotCoordinates(iPose(1:3,:),rot(iPose(4:6,1))) % plots the trajectory as axes
end
xlabel('x')
ylabel('y')
xlabel('z')
end