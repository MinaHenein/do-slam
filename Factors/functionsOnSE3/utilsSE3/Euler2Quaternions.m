function q=Euler2Quaternions(r,p,y)
%transforms Euler angles to quaternions

q=[sin(r/2)*cos(p/2)*cos(y/2)+cos(r/2)*sin(p/2)*sin(y/2);
    cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2);
    cos(r/2)*cos(p/2)*sin(y/2)+sin(r/2)*sin(p/2)*cos(y/2);
    cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2);
    ];
