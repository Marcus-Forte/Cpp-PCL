clear;close all;clc
pc = pcread('cloud_input.pcd');

pcshow(pc,'MarkerSize',50);
hold on
% return
% Least Square for Plane Fitting
%Plane equation -> ax+by+cz = d ~ ax + by + d = z
% Matrix Form XY*[a b c]' = Z

x = pc.Location(:,1);
y = pc.Location(:,2);
z = pc.Location(:,3);

XY = [x y ones(length(x),1)];

abc = inv(XY'*XY)*XY'*z;

n_mesh= 10;
x_mesh = linspace(min(x),max(x),n_mesh)'
y_mesh = linspace(min(y),max(y),n_mesh)'

[x_m y_m] = meshgrid(x_mesh,y_mesh);
z_m = abc(1).*x_m + abc(2).*y_m + abc(3);
surf(x_m,y_m,z_m);


%distance form point p = (x1,y1,z1) from plane (Ax+By+Cz+d=0)
% d = | ax1 + By1 + Cz1 + D|/sqrt(A^2 + B^2 + C^2)
n_points = pc.Count
A = abc(1);B=abc(2);C=-1;D=abc(3);
for i=1:n_points
    x1 = pc.Location(i,1);
    y1 = pc.Location(i,2);
    z1 = pc.Location(i,3);
   d(i) = norm(A*x1 + B*y1 + C*z1 + D)/sqrt(A*A + B*B + C*C);    
end

% Remove Outliers
tolerance = mean(d)*1.5;
count = 1;
for i=1:n_points
    x1 = pc.Location(i,1);
    y1 = pc.Location(i,2);
    z1 = pc.Location(i,3);
   dist = norm(A*x1 + B*y1 + C*z1 + D)/sqrt(A*A + B*B + C*C);    
   if(dist < tolerance)
   xyz_filtered(count,:) = [x1 y1 z1];    
   count = count+1;
   else
       
   end
end
% count
pc_filtered = pointCloud(xyz_filtered)
pcshow(pc_filtered,'markersize',300)

pcwrite(pc_filtered,'cloud_filtered.pcd')




