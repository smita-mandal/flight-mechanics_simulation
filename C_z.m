function out = C_z(A)
% Rotation Matrix about x-axis
row1 = [cos(A), sin(A), 0];
row2 = [-sin(A), cos(A), 0];
row3 = [0, 0, 1];
C_z = [row1; row2; row3];

out = C_z;
end