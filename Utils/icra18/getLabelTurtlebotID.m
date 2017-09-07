function [label, turtlebotID] = getLabelTurtlebotID(s)

label = 0;
turtlebotID = 0;

switch s
    case 'brg'
        label = 1;
        turtlebotID = 1;
    case 'gbr'
        label = 2;
        turtlebotID = 1;
    case 'grb'
        label = 3;
        turtlebotID = 2;
    case 'bgr'
        label = 4;
        turtlebotID = 2;
    case 'rgb'
        label = 5;
        turtlebotID = 0;
end

end