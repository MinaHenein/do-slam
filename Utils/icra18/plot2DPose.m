function plot2DPose(objectPose,color)

plot(objectPose(1,:),objectPose(2,:),color)
hold on
for i=1:size(objectPose,2)
    scatter(objectPose(1,i),objectPose(2,i),color,'o')
    hold on
    endPointX = objectPose(1,i)+0.2*cos(objectPose(6,i));
    endPointY = objectPose(2,i)+0.2*sin(objectPose(6,i));
    plot([objectPose(1,i) endPointX], [objectPose(2,i) endPointY],color)
end

end
