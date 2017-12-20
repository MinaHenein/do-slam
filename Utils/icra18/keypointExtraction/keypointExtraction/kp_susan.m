function points = kp_susan(img)
    % Extract keypoints using SUSAN algorithm
    %
    % Author :: Vincent Garcia
    % Date   :: 05/12/2007
    %
    % INPUT
    % =====
    % img    : the graylevel image
    %
    % OUTPUT
    % ======
    % points : the interest points extracted
    %
    % REFERENCES
    % ==========
    % Smith, S. M. & Brady, J. M. SUSAN - A New Approach to Low Level Image
    % Processing IEEE Transactions Pattern Analysis Machine Intelligence,
    % Kluwer Academic Publishers, 1997, 23, 45-78
    %
    % EXAMPLE
    % =======
    % points = kp_susan(img)
	points = susan(flipud(img'),'-c','-3');
end