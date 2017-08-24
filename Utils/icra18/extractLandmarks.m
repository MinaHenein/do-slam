i=19;
rectified=0;
display =1;

I1 = imread(strcat('/home/mina/Pictures/',num2str(i),'.jpg'));
[points1,label1] = extractLabelLandmarks(I1,12,35);
if(size(label1,1)<6)
    disp('Image needs rectification')
    rectified=1;
    I = imread('/home/mina/Pictures/matlab.jpg');
    if(size(I1,1)~=size(I,1) || size(I1,2)~=size(I,2))
        I1=imcrop(I1,[round((size(I1,2)-size(I,2))/2) round((size(I1,1)-size(I,1))/2)...
            size(I,2)-1 size(I,1)-1]);
    end
    figure,image(I1);
    hold on
    h=imshow(I);
    set(h,'AlphaData',0.5);
    I1=double(I1);
    [I2,H]=frontoparallel(I1);
    [points2,label2] = extractLabelLandmarks(I2,12,60);
end

if size(label2,1)< size(label1,1)
    rectified=0;
end

if display
    if rectified
        points=points2;
        label=label2;
        I=I2;
        labeledPoints=zeros(4,size(points,2));
        for i=1:size(points,2)
            newPoint = H\[points(1:2,i);1];
            newPoint = newPoint/newPoint(3);
            labeledPoints(1:2,i) =   newPoint(1:2);
            labeledPoints(3:4,i) =   points(3:4,i);
            figure;
            imshow(I1);
            hold on
            plotsiftframe(labeledPoints(:,i));
            title(strcat('label ',num2str(label(i))))
            hold off       
        end
    else
        points=points1;
        label=label1;
        I=I1;
    end
    for j=1:size(points,2)
        figure;
        imshow(I);
        hold on
        plotsiftframe(points(:,j));
        title(strcat('label ',num2str(label(j))))
        hold off
    end
    
    
end