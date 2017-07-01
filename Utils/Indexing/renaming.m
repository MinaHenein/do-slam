% Get all PDF files in the current folder
files = dir('*.png');
% Loop through each
for id = 1:length(files)
    % Get the file name (minus the extension)
    [~, f] = fileparts(files(id).name);
    % Get no
    C = strsplit(f,'_');
    imgNo = str2num(C{2});
    newFileName = strcat('Trial_',sprintf('%03d.png', imgNo));
    if ~isnan(imgNo)
        % If numeric, rename
        movefile(files(id).name,newFileName);
    end
end