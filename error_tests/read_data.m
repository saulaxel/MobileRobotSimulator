
fileID = fopen('./test_angle.dat');
num_columnas = 2;
header = textscan(fileID, '%s %s', 1);
data = textscan(fileID, '%f %f');
fclose(fileID);

% Mostramos los datos
header
data
