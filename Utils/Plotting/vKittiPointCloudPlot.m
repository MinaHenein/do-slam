filepath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/staticDynamic20ImagesGT.graph';
fileID = fopen(filepath,'r');
poses = [];
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX_POSE')),'VERTEX_POSE')
        splitLine = strsplit(line,' ');
        poses = [poses, str2double(splitLine(3:end))'];
    end
    line = fgetl(fileID);
end
fclose(fileID);

for k = 1:size(poses,2)
% Camera Transformation Matrix
% R = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];
cameraPose = [rot(poses(4:6,k)), poses(1:3,k); 0 0 0 1];
cameraToWorld = cameraPose;

% Camera Intrinsics
K = [725,      0,     620.5;
       0,    725,     187.0;
       0,      0,        1];

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

rgbFilePath = '/home/mina/Downloads/vKitti/rgbImages';
depthFilePath = '/home/mina/Downloads/vKitti/depthImages';

imgID = 333+k;
I = imread(strcat(rgbFilePath,'/00',num2str(imgID),'.png'));
depth = imread(strcat(depthFilePath,'/00',num2str(imgID),'.png'));

[imgH, imgW, ~] = size(I);

nPoints=imgH*imgW;
PWorld = zeros(nPoints,3);
pcloud = zeros(imgH,imgW,3);
ptCount = 0;


for i=1:imgH
    for j=1:imgW
        
        Z = depth(i,j)/100;
        
        if(Z > 150)
            Z = 0;
        end
        
        X = (j-cx) * Z  / fx;
        Y = (i-cy) * Z  / fy;
        
        ptCount = ptCount + 1;
        P3D =  cameraToWorld*double([X;Y;Z;1]);
        PWorld(ptCount,:) = P3D(1:3,1)';
        pcloud(i,j,:) = PWorld(ptCount,:);
        
    end
end


ptCloud = pointCloud(pcloud,'Color',I);
if k==1
    figure;
end
pcshow(ptCloud);
xlabel('x');
ylabel('y');
zlabel('z');
% axis([-inf inf -inf inf -10 300])
% view([1 1 1])
hold on
    
end