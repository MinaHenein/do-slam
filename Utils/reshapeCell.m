function [outputCell] = reshapeCell(inputCell,mode)
%RESHAPECELL reshapes cell arrays between columns and arrays
%   mode = 'array': Nx1 cell array with elements of size 1xM converted into 
%   cell array of size NxM
%   mode = 'column': NxM cell array converted to Nx1 cell array with 
%   elements of size 1xM

switch mode
    case 'column'
        nRows = size(inputCell,1);
        outputCell = cell(nRows,1);
        for i = 1:nRows
            outputCell{i} = inputCell(i,:);
        end
    case 'array'
        rowLengths = cellfun(@length,inputCell);
        if numel(unique(rowLengths))==1
            nRows = size(rowLengths,1);
            outputCell = cell(nRows,rowLengths(1));
            for i = 1:nRows
                outputCell(i,:) = inputCell{i,:};
            end
        else
            error('inputCell cannot be reshaped. row lengths vary.')
        end
end

end

