function [I2,H1] = frontoparallel(I1)
[rr cc D]=size(I1);
I2=uint8(zeros(rr,cc,D));
[x,y]=ginput(4); 
[xd,yd]=ginput(4);
Y1=[x(1) y(1) 1;x(2) y(2) 1;x(3) y(3) 1;x(4) y(4) 1]';
Y2=[xd(1) yd(1) 1;xd(2) yd(2) 1;xd(3) yd(3) 1;xd(4) yd(4) 1]';
H1=my_homography(Y1,Y2);

% Backward Warping with Bilinear Interpolation
for row=1:rr
    for col=1:cc
coords = H1\[col row 1]';
a=coords(1)-floor(coords(1));
b=coords(2)-floor(coords(2));
coords = floor(coords ./ coords(3));        
initial_row =coords(2);
initial_col=coords(1);
if (initial_row >=1 && initial_row <= size(I1,1)-1 &&  initial_col >= 1 && initial_col <= size(I1,2)-1 )
         I2(row,col,:)=floor((1-a)*(1-b)*I1(coords(2),coords(1),:)+ a*(1-b)*I1(coords(2)+1,coords(1),:)+...
         a*b*I1(coords(2)+1,coords(1)+1,:)+(1-a)*b*I1(coords(2),coords(1)+1,:));
end
   end
end