close all
clear
cap = pcread('33_HDR.ply');
%figure
%showPointCloud(cap);
maxDistance = .73;
roi = [-20,10;-40,40;-15,20];
sdev = 0.73;
decaped = pcdenoise(cap,'Threshold',sdev);

indices = findPointsInROI(decaped, roi);close all

decaped = select(decaped,indices);

gridStep = 0.2;
downsampled = pcdownsample(decaped,'gridAverage',gridStep);


[model1,inlierIndices,outlierIndices] = pcfitplane(downsampled,maxDistance);
bottom = select(downsampled,inlierIndices); %select BOTTOM ground pc
normaltobottom = abs(model1.Normal);
zaxis = [0 0 1];
a = atan2(norm(cross(normaltobottom,zaxis)), dot(normaltobottom,zaxis));

rotate = [
1   0   0   0;
0   cos(a)  sin(a)  0;
0   -sin(a) cos(a)  0;
0   0   0   1;
];


tform = affine3d(rotate);
ptCloudTformed = pctransform(downsampled,tform);


%pcwrite(decaped,'data6_denoised','PLYFormat','binary');
%pcwrite(downsampled,'data9_denoised','PLYFormat','binary');
pcwrite(downsampled,'data9_transformed','PLYFormat','binary');

close all;
figure
pcshow(ptCloudTformed);
title('Transformed')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
ax = gca;
ax.Color = 'yellow';

figure;
pcshow(downsampled);
title('After downsampling')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
ax = gca;
ax.Color = 'yellow';