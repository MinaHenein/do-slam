
function plotColorBar(a)
% figure('units','normalized','position',[0.3750 0.4074 0.2969 0.2130],'color','w');
for i=1:length(a)-1
    switch i
        case 1; color = rgb('light blue');
        case 2; color = rgb('blue');
        case 3; color = rgb('dark blue');
        case 4; color = rgb('light red');
        case 5; color = rgb('red');
        case 6; color = rgb('dark red');
    end
f = [1 2 3 4];
y1 = 0;
y2 = 1;
v = [a(i) y1; a(i+1) y1;...
    a(i+1) y2; a(i) y2];
patch('Faces',f,'Vertices',v,'FaceColor',color);
hold on
end

x1 = 0;
x2 = max(a);
axis([x1 x2 0 20])
ax = gca;
ax.Box = 'on';
ax.YTickLabel = [];
ax.YTick = [];
xlabel('error intervals')
end