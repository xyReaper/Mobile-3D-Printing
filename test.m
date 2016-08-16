clear
strPath = 'data';
strName = '40.ply';
strFull = fullfile(strPath,strName);
cap = pcread(strFull);

%figure
%showPointCloud(cap);
maxDistance = .73;
roi = [-60,20;-40,40;-15,20];
sdev = 0.73;
decaped = pcdenoise(cap,'Threshold',sdev);

indices = findPointsInROI(decaped, roi);close all

decaped = select(decaped,indices);

gridStep = 0.2;
downsampled = pcdownsample(decaped,'gridAverage',gridStep);


[model1,inlierIndices,outlierIndices] = pcfitplane(downsampled,maxDistance);
bottom = select(downsampled,inlierIndices); %select BOTTOM ground pc
normaltobottom = abs(model1.Normal);

%%%%%%%%%%%%rotate to remove camera angle%%%
zaxis = [0 0 1];
a = atan2(norm(cross(normaltobottom,zaxis)), dot(normaltobottom,zaxis));

tran1 = [
1   0   0   0;
0   cos(a)  sin(a)  0;
0   -sin(a) cos(a)  0;
0   0   0   1;
];
angtransf = affine3d(tran1);
ptCloudTformed = pctransform(downsampled,angtransf);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%rotate to align with printer axis%%%
theta = deg2rad(-240);

tran2 = [
cos(theta)  -sin(theta) 0   0;
sin(theta)  cos(theta)  0   0;
0   0   1   0
0   0   0   1;
];
rottranf = affine3d(tran2);
ptCloudTformed = pctransform(ptCloudTformed,rottranf);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

strPath = 'data';
strName = 'data10_transformed';
strFull = fullfile(strPath,strName);

%pcwrite(decaped,'data6_denoised','PLYFormat','binary');
%pcwrite(downsampled,'data9_denoised','PLYFormat','binary');
pcwrite(ptCloudTformed,strFull,'PLYFormat','binary');

close all;
% 
% figure;
% pcshow(downsampled);
% title('After downsampling')
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% ax = gca;
% ax.Color = 'yellow';

figure
pcshow(ptCloudTformed);
title('Transformed')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
ax = gca;
ax.Color = 'yellow';