Image00059Rgb = imread('00000.png');

% camera instrinsics
K = [725 0 620.5;0 725 187;0 0 1];

V = [-0.265882 0.0721791 -0.9612997 7.760932;
0.01772801 0.9973904 0.06998566 112.3845;
0.9638427 0.001565997 -0.2664678 -0.9046764];

YPR_Rad = [-1.571205; 0.001540535; -0.07224254];

WHL = [1.85; 1.50992; 4.930564];
XYZ = [2.904048; 1.4341; 6.406569];


Ry = eul2Rot([0, YPR_Rad(1) + pi/2, 0]);
Rx = eul2Rot([0, 0, YPR_Rad(2)]);
Rz = eul2Rot([YPR_Rad(3), 0, 0]);

% first rotate yaw, then pitch then roll 
R = Ry * Rx * Rz;

% Get bbox corners in object space
x_corners = [WHL(1)/2 WHL(1)/2 -WHL(1)/2 -WHL(1)/2 WHL(1)/2 WHL(1)/2 -WHL(1)/2 -WHL(1)/2];
y_corners = [0 0 0 0 -WHL(2) -WHL(2) -WHL(2) -WHL(2)];
z_corners = [WHL(3)/2 -WHL(3)/2 -WHL(3)/2 WHL(3)/2 WHL(3)/2 -WHL(3)/2 -WHL(3)/2 WHL(3)/2];

corners = [x_corners; y_corners; z_corners];

% Get bbox corners in camera space
pts_3D = R * corners;
pts_3D(1,:) = pts_3D(1,:) + XYZ(1);
pts_3D(2,:) = pts_3D(2,:) + XYZ(2);
pts_3D(3,:) = pts_3D(3,:) + XYZ(3);

% Reprojection in screen space
pts_2D = K * pts_3D;

for c = 1:size(pts_2D,2)
    pts_2D(1,c) = pts_2D(1,c)/pts_2D(3,c);
    pts_2D(2,c) = pts_2D(2,c)/pts_2D(3,c);
end

InputScreenCoordinates = K * XYZ / XYZ(3);

% plot
figure;
imshow(Image00059Rgb);
hold on
plot(InputScreenCoordinates(1),InputScreenCoordinates(2),'gx');
hold on
for c = 1:size(pts_2D,2)
    pts_2D(1,c);
    plot(pts_2D(1,c),pts_2D(2,c),'ro');
end

hold off

