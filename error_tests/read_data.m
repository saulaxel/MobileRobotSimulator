
% Read angle data
fileID = fopen('./test_angle.dat');
header = textscan(fileID, '%s %s', 1);
data = textscan(fileID, '%f %f');
fclose(fileID);

% Show read data
header
data

% Read advance data
fileID = fopen('./test_advance.dat');
header = textscan(fileID, '%s %s %s %s', 1);
data = textscan(fileID, '%f %f %f %f');
fclose(fileID);

% Show read data
header
data


