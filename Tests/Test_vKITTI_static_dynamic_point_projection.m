intrinsics = [725,620.5,187];
R = [0,0,1,0; -1,0,0,0; 0,-1,0,0; 0,0,0,1];

l11 = R*[255.1105,-111.4062,86.7984,1.0000]';
l12 = R*[255.9853,-111.3985,86.3503,1.0000]';
l13 = R*[256.8601,-111.3907,85.9091,1.0000]';

%ls = [77.3516,-252.5883,113.0985]';

ls = [R*[250.2206 -110.9789 85.0406 1]',R*[247.3564 -110.9872 80.2847 1]',...
    R*[240.8085 -113.7841 124.7152 1]',R*[243.6779 -110.986 84.1919 1]',...
    R*[237.4925 -112.6164 126.2936 1]',R*[242.8699 -110.9666 99.3569 1]',...
    R*[250.8672 -110.9891 76.3927 1]',R*[294.9616 -119.7727 117.8011 1]',...
    R*[247.636 -110.9897 78.6398 1]'];



I1 = imread('00380.png');
I2 = imread('00381.png');
I3 = imread('00382.png');

gtPose1 = [71.9792 -242.7422 112.65 0.034511 0.0011743 -0.51507]';
gtPoint1 = l11(1:3);
gtPixel1 = AbsoluteToRelativePositionR3xso3Image(gtPose1,gtPoint1,intrinsics, R);
figure;
imshow(I1);
hold on;
plot(gtPixel1(1),gtPixel1(2),'r*');
hold on;
for i=1:size(ls,2)
    gtPixels = AbsoluteToRelativePositionR3xso3Image(gtPose1,ls(1:3,i),intrinsics, R);
    plot(gtPixels(1),gtPixels(2),'b*');
end



           
gtPose2 = [72.2394 -242.888 112.6499 0.033956 0.0024753 -0.51873]';
gtPoint2 = l12(1:3);
gtPixel2 = AbsoluteToRelativePositionR3xso3Image(gtPose2,gtPoint2,intrinsics, R);

figure;
imshow(I2)
hold on;
plot(gtPixel2(1),gtPixel2(2),'r*');
hold on;
for i=1:size(ls,2)
    gtPixels = AbsoluteToRelativePositionR3xso3Image(gtPose2,ls(1:3,i),intrinsics, R);
    plot(gtPixels(1),gtPixels(2),'b*');
end



gtPose3 = [72.481 -243.0246 112.65 0.033915 0.0034635 -0.52175]';
gtPoint3 = l13(1:3);
gtPixel3 = AbsoluteToRelativePositionR3xso3Image(gtPose3,gtPoint3,intrinsics, R);

figure;
imshow(I3)
hold on;
plot(gtPixel3(1),gtPixel3(2),'r*');
hold on;
for i=1:size(ls,2)
    gtPixels = AbsoluteToRelativePositionR3xso3Image(gtPose3,ls(1:3,i),intrinsics, R);
    plot(gtPixels(1),gtPixels(2),'b*');
end


