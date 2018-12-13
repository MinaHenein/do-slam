function intrinsicMatrix = intrinsicsVecToMatrix(intrinsics)

f  = intrinsics(1);
cx = intrinsics(2);
cy = intrinsics(3);

intrinsicMatrix = [f 0 cx; 0 f cy; 0 0 1];

end