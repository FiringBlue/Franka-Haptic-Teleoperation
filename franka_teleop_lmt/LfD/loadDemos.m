%% Import data from text file.

function pos = loadDemos(filename)
delimiter = '\t';
startRow = 2;

%% Format string for each line of text:
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

idx = find(dataArray{1,11},1,'first');

% Follower robot position x,y,z
pos = [dataArray{1,5}(idx:end,:) dataArray{1,6}(idx:end,:) dataArray{1,7}(idx:end,:)];


%% Close the text file.
fclose(fileID);


end