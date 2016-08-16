ptCloud = pcread('data6_denoised.ply');

ptCloud.Normal = pcnormals(ptCloud);
maxDistance = 0.02;

figure
pcshow(ptCloud)
title('Estimated Normals of Point Cloud')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
hold on

% x = ptCloud.Location(1:100:end,1);
% y = ptCloud.Location(1:100:end,2);
% z = ptCloud.Location(1:100:end,3);
% u = ptCloud.Normal(1:100:end,1);
% v = ptCloud.Normal(1:100:end,2);
% w = ptCloud.Normal(1:100:end,3);
% 
% quiver3(x,y,z,u,v,w);
platform = pcfitplane(ptCloud, maxDistance);

plot(platform)
hold off

vect = platform.Normal;
vect = -1.*vect;
%%%%%%%%%%


referenceVector = [0,0,1];
maxAngularDistance = 5;

[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);

[model3,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,vect,20);
plane3 = select(ptCloud,inlierIndices);


figure
pcshow(plane1)

title('1st plane')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')


figure
pcshow(plane3)
hold on
plot(model3)
hold off


title('soemthing')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
