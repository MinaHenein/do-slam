clear
 P1 = randn(3,1); P2 = randn(3,1); P3 = randn(3,1); % The triangle
 N = cross(P2-P1,P3-P1); % The normal to plane of the triangle
 n = 0; ne = 8192;
 X = zeros(ne,1); Y = zeros(ne,1); Z = zeros(ne,1);
 while n<ne
  Q1 = randn(3,1); Q2 = randn(3,1); % Generate a new line each time
  P0 = Q1 + dot(P1-Q1,N)/dot(Q2-Q1,N)*(Q2-Q1); % Get intersection
  if dot(cross(P2-P1,P0-P1),N)>=0 & ...  % Is P0 is inside triangle?
     dot(cross(P3-P2,P0-P2),N)>=0 & ...
     dot(cross(P1-P3,P0-P3),N)>=0
   n = n+1;
   X(n) = P0(1); Y(n) = P0(2); Z(n) = P0(3); % If so, store P0
  end
 end
 plot3([P1(1),P2(1),P3(1),P1(1)],[P1(2),P2(2),P3(2),P1(2)],...
       [P1(3),P2(3),P3(3),P1(3)],'m-',...
       P1(1),P1(2),P1(3),'ro',P2(1),P2(2),P2(3),'go',...
       P3(1),P3(2),P3(3),'bo',...
       X,Y,Z,'y.')