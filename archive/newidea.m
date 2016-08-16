clear;
ptCloud = pcread('data6_denoised.ply'); %this is Cloud

maxDistance = .73;
maxAngularDistance = 2;
sdevone = 1;

[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance);
bottom = select(ptCloud,inlierIndices); %select BOTTOM ground pc
rem1 = select(ptCloud,outlierIndices); %select Cloud - bottom pc

rem2 = pcdenoise(rem1,'Threshold',sdevone); %denoising 

%no platform
[model2,inlierIndices,outlierIndices] = pcfitplane(rem2,maxDistance);
top = select(rem2,inlierIndices);% select TOP ground pc
rem3 = select(rem2,outlierIndices); %select Cloud - Bottom - top

[model3,inlierIndices,outlierIndices] = pcfitplane(rem3,maxDistance);
ramp = select(rem3,inlierIndices); % select ramp pc

%rem4 = select(rem3,outlierIndices); %select Cloud - bottom - Top - ramp = garbage
% 
% ramp.Normal = pcnormals(ramp); %find all normals to ramp
% slopenor = mean(ramp.Normal); %find the mean normal vector
% 
% sdevtwo = .5;
% rampnew = pcdenoise(ramp,'Threshold',sdevtwo,'NumNeighbors',50);
% 
% [model4,inlierIndices,outlierIndices] = pcfitplane(rampnew,maxDistance,slopenor,maxAngularDistance);
% superramp = select(rampnew,inlierIndices); % select new superramp pc based on refiend normal 
% %rem4 = select(rem3,outlierIndices); %select remaining garbage


[point,line] = plane_intersect(model2,model3);

syms t;
xt = point(1) + t*line(1);
yt = point(2) + t*line(2);
zt = point(3) + t*line(3);

%%%This is for the fitting
% x = superramp.Location(:,1);
% y = superramp.Location(:,2);
% z = superramp.Location(:,3);
x = top.Location(:,1);
y = top.Location(:,2);
z = top.Location(:,3);
x = double(x);
y = double(y);
z = double(z);

sf = fit([x,y],z,'cubicinterp');

[wrapped] = wherestheline(t,xt,yt,zt,x,y,z);

% out = squareform(pdist(wrapped));
% [M,I]=max(out);
% [I_row, I_col] = ind2sub(size(out),I);


dist =0;
[star,fin] = size(wrapped);

for indone = 1:star
    for indtwo = 1:star
        distance = sqrt((wrapped(indone,1)-wrapped(indtwo,1))^2 +(wrapped(indone,2)-wrapped(indtwo,2))^2 +(wrapped(indone,3)-wrapped(indtwo,3))^2 );
        if distance > dist
            eye = indone;
            jay = indtwo;
            dist = distance;
        end
    end
end

midpoint = [0.5*(wrapped(eye,1)+wrapped(jay,1)) 0.5*(wrapped(eye,2)+wrapped(jay,2)) 0.5*(wrapped(eye,3)+wrapped(jay,3))];

objectz = planedistance(model1,model2);



close all;
%%%%%%%%draw stuff###
figure
pcshow(ptCloud)
title('The Point Cloud')
hold on
ax = gca;
ax.Color = 'yellow';
hold off

figure
pcshow(bottom)
title('Bottom')
hold on
plot(model1)
ax = gca;
ax.Color = 'yellow';
hold off

figure
pcshow(rem1)
title('!platform')%
hold on
ax = gca;
ax.Color = 'yellow';
hold off

figure
pcshow(top)
title('top')%
hold on
ax = gca;
ax.Color = 'yellow';
plot(model2)
hold off


figure
pcshow(ramp)
title('slope')%
hold on
ax = gca;
ax.Color = 'yellow';
plot(model3)
hold off


% figure
% pcshow(ramp)
% title('super slope')%
% hold on
% ax = gca;
% plot(super3)
% ax.Color = 'yellow';
% hold off

figure
pcshow(rem1)
title('before denoise')%
hold on
ax = gca;
ax.Color = 'yellow';
hold off

% 
% figure
% pcshow(rem2)
% title('after denoise')%
% hold on
% ax = gca;
% ax.Color = 'yellow';
% hold off
% 
% figure
% pcshow(superramp)
% title('two planes and line intersect')%
% hold on
% pcshow(top)
% plot(model2, 'color', 'blue')
% plot(model4, 'color', 'magenta')
% fp = fplot3(xt,yt,zt,[-50,50],'r--o');
% ax = gca;
% ax.Color = 'yellow';
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% hold off

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
pcshow(top)
title('two planes and line intersect')%
hold on
pcshow(ramp)
mad = plot(model2, 'color', 'blue');
mod = plot(model3, 'color', 'magenta');
fp = fplot3(xt,yt,zt,[-50,50],'r:');
scatter3(wrapped(:,1),wrapped(:,2),wrapped(:,3),'b*');
scatter3(midpoint(1),midpoint(2),midpoint(3),'go');

ax = gca;
ax.Color = 'yellow';
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')

axis auto
hold off
alpha(mad,.2)
alpha(mod,.2)



%%%%%%%%%%%%%%%%%%

figure
pcshow(ramp)
title('two planes and line intersect')%
hold on
pcshow(bottom)
mad = plot(model1, 'color', 'blue');
mod = plot(model3, 'color', 'magenta');
fp = fplot3(xt,yt,zt,[-50,50],'r:');
scatter3(wrapped(:,1),wrapped(:,2),wrapped(:,3),'b*');
scatter3(midpoint(1),midpoint(2),midpoint(3),'go');

ax = gca;
ax.Color = 'yellow';
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')

axis auto
hold off
alpha(mad,.2)
alpha(mod,.2)
%%%%%%%%%%%%%%%%%display normals to something%%%%%%%%%
% 
% figure
% pcshow(ramp)
% title('Estimated Normals of Point Cloud')
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% hold on
% 
% x = ramp.Location(1:1:end,1);
% y = ramp.Location(1:1:end,2);
% z = ramp.Location(1:1:end,3);
% u = ramp.Normal(1:1:end,1);
% v = ramp.Normal(1:1:end,2);
% w = ramp.Normal(1:1:end,3);
% 
% quiver3(x,y,z,u,v,w);
% quiver3(10,0,33,slopenor(1),slopenor(2),slopenor(3))
% hold off
