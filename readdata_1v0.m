close all
clear
strPath = 'data';
strName = '48_HDR.ply';
strFull = fullfile(strPath,strName);
cap = pcread(strFull);

%figure
%showPointCloud(cap);
maxDistance = .73;
roi = [-50,50;-50,50;-15,20];
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
xaxis = [1 0 0];

a = atan2(norm(cross(normaltobottom,zaxis)), dot(normaltobottom,zaxis));
% b = atan2(norm(cross(normaltobottom,xaxis)), dot(normaltobottom,xaxis));

tranx = [
    1   0   0   0;
    0   cos(a)  sin(a)  0;
    0   -sin(a) cos(a)  0;
    0   0   0   1;
    ];%matrix to rotate about x axis

% trany = [
%     cos(b)  0   sin(b)  0;
%     0   1   0   0;
%     -sin(b) 0   cos(b)  0;
%     0   0   0   1;
%     ];%matrix to rotate about y axis


angtransf = affine3d(tranx);
ptCloudTformed = pctransform(downsampled,angtransf);

% angtransf = affine3d(trany);
% ptCloudTformed = pctransform(ptCloudTformed,angtransf);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%rotate to align with printer axis%%%
theta = deg2rad(-240);

tranz = [
cos(theta)  -sin(theta) 0   0;
sin(theta)  cos(theta)  0   0;
0   0   1   0
0   0   0   1;
];%% matrix to transform about z axis
rottranf = affine3d(tranz);
ptCloudTformed = pctransform(ptCloudTformed,rottranf);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

strPath = 'data';
strName = 'data18_transformed';
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