% surface shading and lighting
clear
clc
close

[x,y,z]=sphere(100);
x(60:101,60:101)=NaN;
y(60:101,60:101)=NaN;
z(60:101,60:101)=NaN;

surf(x,y,z)
view(-130,30)
axis vis3d

% material metal
brighten(0.6)
h=light;
% h.Color=[0.5 0.6 1];
%  h.Color=[1 1 1];
h.Position=[-4 -4 10];
h.Style='infinite';%local low beam

 % creates a light source at the starting point
