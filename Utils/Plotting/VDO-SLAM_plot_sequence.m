
sequence = '0001';
file_path = strcat('/home/mina/Downloads/CubeSLAM_map_output/',sequence,'/dynamic_slam_graph_after_opt.g2o');

fid = fopen(file_path);
data = textscan(fid,'%s','delimiter','\n','whitespace',' ');
result_cell = data{1};
fclose(fid);
index_cell = strfind(result_cell, 'EDGE_SE3_MOTION');
motion_index = find(not(cellfun('isempty', index_cell)));

dynamic_point_vertices = [];
motion_vertices = [];
object_points = {};
seen_objects = [];
for i=1:length(motion_index)
    line = str2double(strsplit(result_cell{motion_index(i),1},' '));
    point1_vertex = line(2);
    point2_vertex = line(3);    
    motion_vertex = line(4);
    dynamic_point_vertices = [dynamic_point_vertices, point1_vertex, point2_vertex];
    motion_vertices = [motion_vertices, motion_vertex];
    if ~ismember(motion_vertex, seen_objects)
        seen_objects = [seen_objects, motion_vertex];
        indx = find(seen_objects == motion_vertex);
        object_points{indx,1} = point1_vertex;
        object_points{indx,1} = [object_points{indx,1},point2_vertex];
    else
        indx = find(seen_objects == motion_vertex);
        if ~ismember(point1_vertex, [object_points{indx,1}])
            object_points{indx,1} = [object_points{indx,1},point1_vertex];
        end
        if ~ismember(point2_vertex, [object_points{indx,1}])
            object_points{indx,1} = [object_points{indx,1},point2_vertex];
        end
    end
end 

dynamic_point_vertices = unique(dynamic_point_vertices);
motion_vertices = unique(motion_vertices);

index_cell = strfind(result_cell, 'EDGE_SE3:QUAT');
smoothing_index = find(not(cellfun('isempty', index_cell)));
unique_motion_vertices = {};
for i=1:length(smoothing_index)
    line = str2double(strsplit(result_cell{smoothing_index(i),1},' '));
    vertex1 = line(2);
    vertex2 = line(3);    
    if ismember(vertex1, motion_vertices) && ismember(vertex2, motion_vertices)   
        if isempty(unique_motion_vertices)
            unique_motion_vertices{1,1} = [vertex1,vertex2];
        else
            present = 0;
            for j = 1:size(unique_motion_vertices,1)
                if ismember(vertex1, [unique_motion_vertices{j,1}])
                    present = 1;
                    unique_motion_vertices{j,1} = [unique_motion_vertices{j,1},vertex2];
                end
            end
            if ~present
                unique_motion_vertices{size(unique_motion_vertices,1)+1,1} = [vertex1,vertex2];
            end
        end
    end
end 

config = CameraConfig();
config = setAppConfig(config);
convertG2OGraphFileToOurs(config,file_path,motion_vertices)

[~,~,ext] = fileparts(file_path);
fid = fopen(strcat(file_path(1:end-length(ext)),'G2O.graph'));
data = textscan(fid,'%s','delimiter','\n','whitespace',' ');
result_cell = data{1};
fclose(fid);
result_poses = [];
result_points = [];
result_poses_vertices = [];
result_point_vertices = [];
for i = 1:size(result_cell,1)
    line = strsplit(result_cell{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        result_poses = [result_poses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
        result_poses_vertices = [result_poses_vertices , str2double(line{2})];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        result_points = [result_points, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
        result_point_vertices = [result_point_vertices , str2double(line{2})];
    end
end

clear h
figure; hold on;
for i=1:5:size(result_poses,2)
    pose = result_poses(:,i);
    plotiCamera = plotCamera('Location',pose(1:3),'Orientation',rot(-pose(4:6))); %LHS invert pose
    plotiCamera.Opacity = 0.1;
    plotiCamera.Size = 0.9;
    plotiCamera.Color = 'red';
end

static_point_vertices = setdiff(result_point_vertices, dynamic_point_vertices);
static_points = zeros(3,length(static_point_vertices));
n = 0;
for i=1:size(result_points,2)
    if ismember(result_point_vertices(i),static_point_vertices)
        n = n+1;
        static_points(:,n) =  result_points(:,i);
    end
end

plot_static_structure = plot3(static_points(1,1:10:end),static_points(2,1:10:end),static_points(3,1:10:end),...
  'Color', rgb('black'),'Marker','.','MarkerSize',7,'LineStyle','none');
h(1) = plot_static_structure;

colors = {'blue','radioactive green','magenta','cyan','cornflower',...
    'sapphire','swamp','plum','light bluish green','butterscotch','cinnamon',...
    'chartreuse','green','blue','raw sienna','baby purple','cocoa','light royal blue',...
    'orangeish','rust brown','sand brown','tealish green','burnt siena','camo',...
    'dusk blue','fern','old rose','pale light green','peachy pink','rosy pink',...
    'light bluish green','light bright green','light neon green','light seafoam',...
    'tiffany blue','washed out green','browny orange','nice blue','sapphire',...
    'greyish teal','orangey yellow','parchment','straw','very dark brown','terracota',...
    'ugly blue','clear blue','creme','foam green','grey/green','light gold','seafoam blue',...
    'topaz','violet pink','wintergreen','yellow tan','dark fuchsia','indigo blue',...
    'light yellowish green','pale magenta','rich purple','sunflower yellow','darkish purple',...
    'true green','coral pink','dark sage','dark slate blue','flat blue','mushroom','rich blue',...
    'dirty purple','greenblue','icky green','light khaki','warm blue','dark hot pink',...
    'deep sea blue','carmine','dark yellow green','pale peach','plum purple','golden rod','neon red','old pink'};

for j = 1:size(object_points,1)
    ids = [object_points{j,1}];
    if ~isempty(ids)
        points = [];
        for m = 1:length(ids)
            points = [points, result_points(:,result_point_vertices == ids(m))];
        end
        for n = 1:size(unique_motion_vertices,1)
            if ismember(seen_objects(j),unique_motion_vertices{n,1})
                plotObject = plot3(points(1,:),points(2,:),points(3,:),'.',...
                    'Color', rgb(colors{n}),'MarkerSize',7, 'Color', rgb(colors{n}),'LineStyle','none');
                h(n+1) = plotObject;
                hold on
            break
            end
        end
    end
end


xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
view(0,0)

fig_legend = {' static structure '};
for i = 1:size(unique_motion_vertices,1)
    fig_legend{i+1} = " object " + " " + num2str(i);
end
l = legend(h, cellstr(fig_legend));
l.Location= 'northwest';
l.FontSize = 15;

AxesH    = gca;
UpVector = [-sind(30), cosd(30), 0];
DAR      = get(AxesH, 'DataAspectRatio');
AxesH.Box = 'on';
AxesH.FontSize = 16;
set(AxesH, 'CameraUpVector', DAR .* UpVector);

