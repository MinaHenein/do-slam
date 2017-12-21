function patternCentroid = findPatternCentroid(img, pt)

Africa = 0;
NorthAmerica = 0;
SouthAmerica = 0;
AsiaEurope= 0;

aCentroid = [0,0];
naCentroid = [0,0];
saCentroid = [0,0];
aeCentroid = [0,0];
centroids = zeros(4,1);

n = 0;
ptIndexes = [];

for i=1:size(pt,1)
    rgb = img(pt(i,1),pt(i,2),:);
    r =  rgb(1,1,1);
    g =  rgb(1,1,2);
    b =  rgb(1,1,3);
    
    if r >=240 && g>=100 && g<=240 && b<=50
        Africa = 1;
        n =  n + 1;
        ptIndexes = [ptIndexes; i,1];
        centroids(1) = 1;
    end
    if r <=30 && g>=90 && g<=180 && b>=160
        NorthAmerica = 1;
        n = n + 1;
        ptIndexes = [ptIndexes; i,2];
        centroids(2) = 1;
    end
    if r >=70 && r<=150 && g>=140 && g<=185 && b>=50 && b<=80
        SouthAmerica = 1;
        n = n + 1;
        ptIndexes = [ptIndexes; i, 3];
        centroids(3) = 1;
    end
    if r >=95 && r<=240 && g>=30 && g<=80 && b>=95 && b<=170
        AsiaEurope= 1;
        n  = n + 1;
        ptIndexes = [ptIndexes; i, 4];
        centroids(4) = 1;
    end
end

if NorthAmerica && AsiaEurope
    na = find(ptIndexes(:,2) == 2);
    ae = find(ptIndexes(:,2) == 4);
    x = 0;
    y = 0;
    for i = 1:length(na)
        x = x + pt(na(i),1);
        y = y + pt(na(i),2);
    end
    naCentroid = [x/length(na),y/length(na)];
    
    x = 0;
    y = 0;
    for i = 1:length(ae)
        x = x + pt(ae(i),1);
        y = y + pt(ae(i),2);
    end
    aeCentroid = [x/length(ae),y/length(ae)];

dist = sqrt((naCentroid(1) - aeCentroid(1))^2+(naCentroid(2) - aeCentroid(2))^2);

end

if dist >=50 && dist <= 90
    if Africa
        a = find(ptIndexes(:,2) == 1);
        x = 0;
        y = 0;
        for i = 1:length(a)
            x = x + pt(a(i),1);
            y = y + pt(a(i),2);
        end
        aCentroid = [x/length(a),y/length(a)];
    end
    if SouthAmerica
        sa = find(ptIndexes(:,2) == 3);
        x = 0;
        y = 0;
        for i = 1:length(sa)
            x = x + pt(sa(i),1);
            y = y + pt(sa(i),2);
        end
        saCentroid = [x/length(sa),y/length(sa)];
    end
end

if n > 2
    patternCentroid = (naCentroid + aeCentroid + aCentroid + saCentroid)/ sum(centroids);
end

end