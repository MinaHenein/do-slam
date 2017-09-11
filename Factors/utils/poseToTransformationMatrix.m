function transformationMatrix = poseToTransformationMatrix(pose)


transformationMatrix = [rot(pose(4:6)) pose(1:3); 0 0 0 1];

end