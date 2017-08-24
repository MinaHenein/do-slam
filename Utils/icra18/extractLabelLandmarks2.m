for i = 17  
    I = imread(strcat('/home/mina/Pictures/',num2str(i),'.jpg'));
    cform = makecform('srgb2lab');
    lab_I = applycform(I,cform);   
    ab = double(lab_I(:,:,2:3));
    nrows = size(ab,1);
    ncols = size(ab,2);
    ab = reshape(ab,nrows*ncols,2);
    nColors = 3;
    [cluster_idx,~] = kmeans(ab,nColors,'distance','sqEuclidean',...
        'Replicates',3);
    pixel_labels = reshape(cluster_idx,nrows,ncols);
    segmented_images = cell(1,3);
    rgb_label = repmat(pixel_labels,[1 1 3]);
    for k = 1:nColors
        color = I;
        color(rgb_label ~= k) = 0;
        segmented_images{k} = color;
    end
    imgColor = zeros(nColors,1);
    for l=1:nColors
        img = segmented_images{l};
        reds = 0;
        blues = 0;
        greens = 0;
        for m = 1:nrows
            for n=1:ncols
                if(img(m,n,1)-img(m,n,2)>65 && img(m,n,1)-img(m,n,3)>65)
                    reds = reds +1;
                elseif(img(m,n,2)-img(m,n,1)>65 && img(m,n,2)-img(m,n,3)>65)
                    greens = greens +1;
                elseif(img(m,n,3)-img(m,n,2)>65 && img(m,n,3)-img(m,n,1)>65)
                    blues = blues +1;
                end
            end
        end
        if(reds>blues && reds>greens)
            imgColor(l,1) = 1;
        elseif(greens>reds && greens>blues)
            imgColor(l,1) = 2;
        elseif(blues>reds && blues>greens)
            imgColor(l,1) = 3;
        end
    end
    % looking for features is best done in the blue segmented image
    siftThreshold = 6;
    points = sift(rgb2gray(segmented_images{imgColor==3}),'Threshold',siftThreshold);
    figure
    imshow(I);
    hold on
    plotsiftframe(points)
    
    imH = size(I,1);
    imW = size(I,2);
    points(1,:) = round(points(1,:));
    points(2,:) = round(points(2,:));
    rightColor = zeros(size(points,2),1);
    leftColor = zeros(size(points,2),1);
    for j=1:size(points,2)
        right = I(points(2,j),min(points(1,j)+50,imW),:);
        left = I(points(2,j),max(points(1,j)-50,1),:);
        if right(1,1,1)-right(1,1,2)>50 && right(1,1,1)-right(1,1,3)>50
            rightColor(j) = 1;          
        elseif right(1,1,2)-right(1,1,1)>50 && right(1,1,2)-right(1,1,3)>50
            rightColor(j) = 2;
        elseif right(1,1,3)-right(1,1,2)>50 && right(1,1,3)-right(1,1,1)>50
            rightColor(j) = 3;
        end
        
        if left(1,1,1)-left(1,1,2)>50 && left(1,1,1)-left(1,1,3)>50
            leftColor(j) = 1;          
        elseif left(1,1,2)-left(1,1,1)>50 && left(1,1,2)-left(1,1,3)>50
            leftColor(j) = 2;
        elseif left(1,1,3)-left(1,1,2)>50 && left(1,1,3)-left(1,1,1)>50
            leftColor(j) = 3;
        end
    end
    label = zeros(size(points,2),1);
    for k=1:size(points,2)
        if rightColor(k)==0 && leftColor(k)==2; label(k) = 1; 
        elseif rightColor(k)==0 && leftColor(k)==1; label(k) = 2;
        elseif rightColor(k)==2 && leftColor(k)==1; label(k) = 3;
        elseif rightColor(k)==2 && leftColor(k)==0; label(k) = 4;
        elseif rightColor(k)==1 && leftColor(k)==0; label(k) = 5;
        elseif rightColor(k)==1 && leftColor(k)==2; label(k) = 6;
        end
    end
    
    points(:,label==0) = [];
    label(~any(label,2),:) = []; 
    
    for h=1:size(points,2)
        figure
        imshow(I);
        hold on
        plotsiftframe(points(:,h));
        title(strcat('label ',num2str(label(h))))
    end
end