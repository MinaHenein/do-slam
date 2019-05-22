sequence = [1,2,6,18];
fps = 10;
for i = 1:4
    Seq = sequence(i);
    if Seq == 1
        %Seq0001:
        imageRange = 334 : 425;
        objectDetectionFile = strcat(pwd,'/Tests/vk-objectPoses/0001_objects.txt');
        cameraExtrinsicsFile = strcat(pwd,'/Tests/vk-objectPoses/0001_camera.txt');
    elseif Seq == 2
        %Seq0002:
        imageRange = 80 : 216;
        objectDetectionFile = strcat(pwd,'/Tests/vk-objectPoses/0002_objects.txt');
        cameraExtrinsicsFile = strcat(pwd,'/Tests/vk-objectPoses/0002_camera.txt');
    elseif Seq == 6
        %Seq0006:
        imageRange = 0 : 170;
        objectDetectionFile = strcat(pwd,'/Tests/vk-objectPoses/0006_objects.txt');
        cameraExtrinsicsFile = strcat(pwd,'/Tests/vk-objectPoses/0006_camera.txt');
    elseif Seq == 18
        %Seq0018:
        imageRange = 25 : 337;
        objectDetectionFile = strcat(pwd,'/Tests/vk-objectPoses/0018_objects.txt');
        cameraExtrinsicsFile = strcat(pwd,'/Tests/vk-objectPoses/0018_camera.txt');
    end
    objectPosesCell = saveGTObjectPoses(imageRange, objectDetectionFile, cameraExtrinsicsFile);
    plotVelocityProfiles(objectPosesCell, fps, Seq);
end
