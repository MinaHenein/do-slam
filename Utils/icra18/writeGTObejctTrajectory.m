function poses = writeGTObejctTrajectory(filePath, pose1, HGt)


poses = zeros(28,6);
poses(1,:) =  pose1;

for i=2:28    
  poses(i,:) = RelativeToAbsolutePoseR3xso3GlobalFrame(poses(i-1,:)',HGt);
end

for i =1 :28
fID = fopen(filePath,'a');
fprintf(fID,'%s %d %6f %6f %6f %6f %6f %6f \n','VERTEX_POSE_R3_SO3',i,poses(i,:));
end

end