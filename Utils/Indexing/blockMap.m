function [varargout] = blockMap(system,varargin)
%BLOCKMAP maps edge and vertex indices to location in system matrix A
%   inputs:
%       system  uses edgeHeights and vertexWidths properties
%       i       edge index
%       j       vertex index
%   outputs:
%       iBlock  rows in system matrix A
%       jBlock	columns in system matrix A

iHeights = system.edgeHeights;
jWidths  = system.vertexWidths;

%do you want edges, vertices or both?
if isnumeric(varargin{1}) && isnumeric(varargin{2})
    i = varargin{1};
    j = varargin{2};
    
    iStart = sum(iHeights(1:i-1)) + 1;
    iEnd   = iStart + iHeights(i) - 1;
    iBlock = iStart:iEnd;

    jStart = sum(jWidths(1:j-1)) + 1;
    jEnd   = jStart + jWidths(j) - 1;
    jBlock = jStart:jEnd;
    
    nargout = 2;  
    varargout{1} = iBlock;
    varargout{2} = jBlock;
else
    switch varargin{2}
        case 'edge'
            i = varargin{1};
            iStart = sum(iHeights(1:i-1)) + 1;
            iEnd   = iStart + iHeights(i) - 1;
            iBlock = iStart:iEnd;
            
            nargout = 1;
            varargout{1} = iBlock;
        case 'vertex'
            j = varargin{1};
            jStart = sum(jWidths(1:j-1)) + 1;
            jEnd   = jStart + jWidths(j) - 1;
            jBlock = jStart:jEnd;
            
            nargout = 1;
            varargout{1} = jBlock;
    end
end

end

