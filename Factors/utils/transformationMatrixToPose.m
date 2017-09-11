function pose = transformationMatrixToPose(transformationMatrix)

pose = [transformationMatrix(1:3,4);arot(transformationMatrix(1:3,1:3))];

end