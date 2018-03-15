filepath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/vKitti_results.graph';
fileID = fopen(filepath,'r');
poses = [];
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX_POSE_R3_SO3')),'VERTEX_POSE_R3_SO3')
        splitLine = strsplit(line,' ');
        poses = [poses, str2double(splitLine(3:end))'];
    end
    line = fgetl(fileID);
end
fclose(fileID);

for k = 1:size(poses,2)
% Camera Transformation Matrix
cameraToWorld = [rot(poses(k,4:6)), poses(k,1:3); 0 0 0 1];

% Camera Intrinsics
K = [329.115520046, 0,             320.0;
    0,             329.115520046,  240.0;
    0,             0,                 1];

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

factor = 1; % 1/pixel to meter conversion factor

filePath = '/home/mina/City Dataset/img';
depthFilePath = '/home/mina/City Dataset/data/depth';

I = imread(strcat(filePath,'/',getFileName(imgID),num2str(imgID),'_0.png'));
% THIS IS SO IMPORTANT TO KEEP RIGHT VALUES OF DEPTH
depth = reshape(load(strcat(depthFilePath,'/',getFileName(imgID),num2str(imgID),'_0.depth')),640, 480)';

[imgH, imgW, ~] = size(I);

nPoints=imgH*imgW;
PWorld = zeros(nPoints,3);
pcloud = zeros(imgH,imgW,3);
ptCount = 0;


for i=1:imgH
    for j=1:imgW
        
        pt = [j;i;1];
        eta = K\pt;
        eta = eta/norm(eta);
        
        Z = depth(i,j)*eta(3) / factor;
        if(Z > 500)
            Z = 0;
        end
        X = (j-cx) * Z  / fx;
        Y = (i-cy) * Z  / fy;
        
        ptCount = ptCount + 1;
        P3D =  cameraToWorld*[X;Y;Z;1];
        PWorld(ptCount,:) = P3D(1:3,1)';
        pcloud(i,j,:) = PWorld(ptCount,:);
        
    end
end


ptCloud = pointCloud(pcloud,'Color',I);
%figure
pcshow(ptCloud);
xlabel('x');
ylabel('y');
zlabel('z');
hold on
    
end