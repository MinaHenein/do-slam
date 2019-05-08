
function framesToVideo(video,videoName,startFrame,endFrame)

framesPath = strcat('/media/mina/JunHDD/Dynamic SLAM Data/',video,'/'); 
fps = 12;  

if(exist('videoName','file'))
    delete videoName.avi
end

aviobj=VideoWriter(videoName);
aviobj.FrameRate=fps;

%Open file for writing video data
open(aviobj);

for i=startFrame:endFrame
    nDigits = numel(num2str(i));
    fileName = strcat(framesPath,'img_',repmat('0',[1,9-nDigits]),num2str(i),'.ppm');
    frame = imread(fileName);
    imwrite(frame,strcat(fileName(1:end-17),'jpg/','img_',...
        repmat('0',[1,9-nDigits]),num2str(i),'.jpg'))
    writeVideo(aviobj,frame);
end
close(aviobj);

end