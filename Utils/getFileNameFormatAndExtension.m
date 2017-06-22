function [fileNameFormat,extension] = getFileNameFormatAndExtension(fileName)

%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 20/06/17
% Contributors:
%--------------------------------------------------------------------------

% returns the format of a file name
% to be used only when asynchronous data where each file name is the time
% stamp at which it was created
% input: fileName
% output: format of file name
% e.g fileName = 12345.123.1234.png
% output fileNameFomart = 00000.000.0000

lastDot = strfind(fileName,'.');
lastDot = lastDot(end);
fileName = fileName(1:lastDot-1);
extension = fileName(lastDot:end);

nDots        = length(strfind(fileName,'.'));
nCommas      = length(strfind(fileName,','));
nColumns     = length(strfind(fileName,':'));
nSlashes     = length(strfind(fileName,'/'));
nUnderscores = length(strfind(fileName,'_'));

nSeparators = [nDots nCommas nColumns nSlashes nUnderscores];
sep = find(nSeparators);

switch sep
    case 1
        separator =  '.';
    case 2
        separator =  ',';
    case 3
        separator =  ':';
    case 4
        separator =  '/';
    case 5
        separator =  '_';
    otherwise
        error('unsupported separator type')
end

assert(length(sep)==1,'Unconventional file name with more than 1 separator type')

line = strsplit(fileName,separator);
nStrings = size(line,2);

fileNameFormat = '';
for i = 1:nStrings
    for j = 1:length(line{i})
        fileNameFormat = strcat(fileNameFormat,'0');
    end
    if(i<= nStrings-1)
    fileNameFormat = strcat(fileNameFormat,separator);
    end
end
end