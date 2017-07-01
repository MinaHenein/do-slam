function [indexes2] = remapIndex(indexes1,map,type)
%REMAPINDEX Summary of this function goes here
%   Detailed explanation goes here

nIndexes = numel(indexes1);
indexes2 = zeros(size(indexes1));

switch type
    case 'Point'
        for i = 1:nIndexes
            indexes2(i) = find([map.points.index]==indexes1(i));
        end
    case 'Entity'
        for i = 1:nIndexes
            indexes2(i) = find([map.entities.index]==indexes1(i));
        end
    case 'Object'
        for i = 1:nIndexes
            indexes2(i) = find([map.objects.index]==indexes1(i));
        end
    case 'Constraint'
        for i = 1:nIndexes
            indexes2(i) = find([map.constraint.index]==indexes1(i));
        end
end





end

