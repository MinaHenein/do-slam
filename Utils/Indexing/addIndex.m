function [list] = addIndex(list,newIndexes,orientation,varargin)
%ADDINDEX %ensures list of indexes unique
          %removes index 0 once another index added
    
    switch orientation
        case 'row'
            if ~isrow(list)
                list = list';
            end
            if ~isrow(newIndexes)
                newIndexes = newIndexes';
            end
            list = [list newIndexes];
        case 'col'
            if isrow(list)
                list = list';
            end
            if isrow(newIndexes)
                newIndexes = newIndexes';
            end
            list = [list; newIndexes];
    end
          
    if nargin==4 && strcmp(varargin{1},'unique')
        list = unique(list);
    end

end

