function H = my_homography(X1,X2)
A=[];
for i = 1:4
    xip = X2( 1, i );
    yip = X2( 2, i );
    wip = X2( 3, i );

    xi = X1( :, i );

    Ai = [ 0, 0, 0,    -wip * xi',   yip * xi' ;
           wip * xi',     0, 0, 0,  -xip * xi' ];

    A = [ A ; Ai ];
 end;
  [U,D,V] = svd( A );
  H = reshape( V(:,9), 3, 3 )';
end