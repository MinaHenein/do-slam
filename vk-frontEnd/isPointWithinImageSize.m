function isWithinImageSize = isPointWithinImageSize(pt, imgSize)

if(round(pt(1,1))<1 ||round(pt(1,1))>imgSize(2) || round(pt(2,1))<1 || round(pt(2,1))>imgSize(1))
    isWithinImageSize = 0;
else
    isWithinImageSize = 1;
end

end