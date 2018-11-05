function vKittiPointCloudPlot(i)

dir = '/home/mina/Downloads/vKitti/';
cameraExtrinsicsFile = strcat(dir,'vkitti_1.3.1_extrinsicsgt/0001_clone.txt');
% camera intrinsics
K = [725,      0,     620.5;
       0,    725,     187.0;
       0,      0,        1];
% read rgb image
rgbI = imread(strcat(dir,'rgbImages/00',num2str(i),'.png'));
% read depth image
depthI = imread(strcat(dir,'depthImages/00',num2str(i),'.png'));
% read camera pose
fid = fopen(cameraExtrinsicsFile);
lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+1);
fclose(fid);
lineArray = str2num(cell2mat(lineCell{1,1}));
assert(lineArray(1)==i);
cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
% preallocate memory
[imgH, imgW, ~] = size(rgbI);
pcloud  = zeros(imgH,imgW,3);

for i=1:imgH
    for j=1:imgW
        pixelRow = i;
        pixelCol = j;
        pixelDepth = double(depthI(pixelRow,pixelCol));
        if pixelDepth/100>400
           continue
        end
        camera3DPoint = K\[pixelRow;pixelCol;1];
        camera3DPoint(2) = -camera3DPoint(2);
        camera3DPoint(3) = pixelDepth/100;
        world3DPoint = cameraPoseMatrix * [camera3DPoint;1];
        pcloud(i,j,:) = world3DPoint(1:3,1);
    end
end

ptCloud = pointCloud(pcloud,'Color',rgbI);

pcshow(ptCloud);
xlabel('x');
ylabel('y');
zlabel('z');
hold on

end