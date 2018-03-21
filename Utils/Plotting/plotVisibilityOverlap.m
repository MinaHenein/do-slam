function plotVisibilityOverlap(sensors)
figure('units','normalized','position',[0.3750 0.4074 0.2969 0.2130],'color','w');
for i=1:length(sensors)
    sensorVisibility = sensors(i).get('pointVisibility');
    [~,col] = find(sensorVisibility);
    sensorFirstPointTime = col(1);
    sensorLastPointTime  = col(end);
    switch i
        case 1; color = 'r';
        case 2; color = 'g';
        case 3; color = 'b';
        case 4; color = 'k';
    end
    f = [1 2 3 4];
    y1(i) = (i-1)*2+ 2*i;
    y2(i) = y1(i)+2;
    v = [sensorFirstPointTime y1(i); sensorLastPointTime y1(i);...
        sensorLastPointTime y2(i); sensorFirstPointTime y2(i)];
    patch('Faces',f,'Vertices',v,'FaceColor',color,'FaceAlpha',0.75);
    hold on
    x1 = 1;
    x2 = 320;
    plot([x1 x2], [1 1]*(y1(i)+y2(i))/2,'k-.');

    

end

    axis([x1 x2 0 20])
    ax = gca;
    ax.Box = 'on';
%     ax.YTickLabel = [];
%     ax.YTick = [];
%     ax.YTickLabelMode = 'manual';
    xlabel('time')
    yticks((y1 + y2)/2);
    yticklabels({'camera 1','camera 2','camera 3','camera 4'})
    
end
