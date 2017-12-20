function points = kp_gilles(im,o_radius)
    % Extract keypoints using Gilles algorithm
    %
    % Author :: Vincent Garcia
    % Date   :: 05/12/2007
    %
    % INPUT
    % =====
    % im       : the graylevel image
    % o_radius : (optional) the radius of blobs detected
    %
    % OUTPUT
    % ======
    % points : the interest points extracted
    %
    % REFERENCES
    % ==========
    % S. Gilles, Robust Description and Matching of Images. PhD thesis,
    % Oxford University, Ph.D. thesis, Oxford University, 1988.
    %
    % EXAMPLE
    % =======
    % points = kp_gilles(im,10) % radius of 10 pixels

    % only luminance value
    im = im(:,:,1);

    % variables
    if nargin==1
        radius = 10;
    else
        radius = o_radius;
    end
    mask = fspecial('disk',radius)>0;

    % compute local entropy
    loc_ent = entropyfilt(im,mask);

    % find the local maxima
    [l,c,tmp] = findLocalMaximum(loc_ent,radius);

    % keep only points above a threshold
    [l,c]     = find(tmp>0.95*max(tmp(:)));
    points    = [l,c,repmat(radius,[size(l,1),1])];

end