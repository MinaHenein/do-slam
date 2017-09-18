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
    case '6'
        label = 6;
        turtlebotID = 0;
    case '7'
        label = 7;
        turtlebotID = 0;
    case '8'
        label = 8;
        turtlebotID = 0;
    case '9'
        label = 9;
        turtlebotID = 0;
    case '10'
        label = 10;
        turtlebotID = 0;
    case '11'
        label = 11;
        turtlebotID = 0;
    case '12'
        label = 12;
        turtlebotID = 0;
end

end