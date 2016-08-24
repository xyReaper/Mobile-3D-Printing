%version 1.2
function [center,angle] = detection(strName)

strPath = 'data';
strFull = fullfile(strPath,strName);

ptCloud = pcread(strFull); %this is Cloud

maxDistance = .9;
maxAngularDistance = 5;
sdevone = 1;

[TopModel,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance);
bottom = select(ptCloud,inlierIndices); %select BOTTOM ground pc
rem1 = select(ptCloud,outlierIndices); %select Cloud - bottom pc

rem2 = pcdenoise(rem1,'Threshold',sdevone); %denoising 
clear rem1; 

%no platform
[BottomModel,inlierIndices,outlierIndices] = pcfitplane(rem2,maxDistance);
top = select(rem2,inlierIndices);% select TOP ground pc
rem3 = select(rem2,outlierIndices); %select Cloud - Bottom - top
clear rem2;

[RampModel,inlierIndices,outlierIndices] = pcfitplane(rem3,maxDistance);
ramp = select(rem3,inlierIndices); % select ramp pc
clear rem3;
%%early checks and sets

%%this will make sure the BottomModel/bottom is pointing vertically up
if (TopModel.Normal(3)<=0.9)
    if (TopModel.Normal(3)<=-0.9)
        holdpar = -1.*(TopModel.Parameters);
        TopModel = planeModel(holdpar);
        clear holdpar
     end
end

%%this will make sure the TopModel/top is pointing vertically up
if (BottomModel.Normal(3)<=0.9)
    if (BottomModel.Normal(3)<=-0.9)
        holdpar = -1.*(BottomModel.Parameters);
        BottomModel = planeModel(holdpar);
        clear holdpar
     end
end

%%this will make sure the RampModel/ramp is pointing away from 3D camera
if (RampModel.Normal(1)<0)
    if (RampModel.Normal(2)>0)
        if (RampModel.Normal(1)<0)
        holdpar = -1.*(RampModel.Parameters);
        RampModel = planeModel(holdpar);
        clear holdpar
        end
     end
end

[point,line] = plane_intersect(TopModel,RampModel);
direction = cross(TopModel.Normal, line);


syms t;
xt = point(1) + t*line(1);
yt = point(2) + t*line(2);
zt = point(3) + t*line(3);

xrangemax= ramp.XLimits(2) + 1;
xrangemin= ramp.XLimits(1) - 1;

yrangemax= ramp.YLimits(2) + 1;
yrangemin= ramp.YLimits(1) - 1;

zrangemax= ramp.ZLimits(1) + 2;
zrangemin= ramp.ZLimits(1) - 2;



roi = [xrangemin,xrangemax;yrangemin,yrangemax;zrangemin,zrangemax];

indices = findPointsInROI(bottom, roi);
search = select(bottom,indices);


x = search.Location(:,1);
y = search.Location(:,2);
z = search.Location(:,3);
x = double(x);
y = double(y);
z = double(z);

% sf = fit([x,y],z,'cubicinterp');
%%%%%%%%%%%%%%%%%
%% PARFOR

[wrapped] = wheres_the_line(t,xt,yt,zt,x,y,z);
%% 

% out = squareform(pdist(wrapped));
% [M,I]=max(out);
% [I_row, I_col] = ind2sub(size(out),I);



dist = 0;
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

mid = [0.5*(wrapped(eye,1)+wrapped(jay,1)) 0.5*(wrapped(eye,2)+wrapped(jay,2)) 0.5*(wrapped(eye,3)+wrapped(jay,3))];

% objectz = planedistance(BottomModel,TopModel);% height between two planes
%%%%%%%%%%%%%%%%%%%

b_3 = 20;
brick_length = (b_3*3); 
printcenter = [-3.264 3.879]; %finetune current print center to 3D scnaner center
%syms tt;
%linetocenter= [(direction(1)*tt + midpoint(1)) (direction(2)*tt + midpoint(2))];
direction = direction./norm(direction);
centerofprev = [(mid(1)-(0.5*brick_length*direction(1)))   (mid(2)-(0.5*brick_length*direction(2)))];

centerofnew = centerofprev + [((2/3)*brick_length*direction(1)) ((2/3)*brick_length*direction(2))];
xaxis = [1  0   0];

angleofrotation = atan2(norm(cross(line,xaxis)), dot(line,xaxis));% find the angle of rotation
% angleofrotation = rad2deg (angleofrotation);
angleofrotation = angleofrotation - pi/2;
centerofnew = centerofnew - printcenter;

center = centerofnew;
angle = angleofrotation;

% disp (centerofnew);
% disp (angleofrotation);
% angleofrotation = rad2deg (angleofrotation);
% disp (angleofrotation);

%sanity check
oldtonew = sqrt((centerofnew(1)-centerofprev(1))^2+(centerofnew(2)-centerofprev(2))^2);
if ((b_3*2-1) <= oldtonew <= (b_3*2+1))
    disp('distances checkout!');
else
    disp('Something looks wrong');
end

%% 
close all;
%%%%%%%%draw stuff###

%% 
figure
pcshow(ramp)
title('two planes and line intersect')%
hold on
ax = gca;
ax.Color = 'yellow';
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')

pcshow(bottom)
pcshow(top)
mad = plot(RampModel, 'color', 'blue');
mod = plot(TopModel, 'color', 'magenta');
mod.FaceAlpha = 0.3;
mad.FaceAlpha = 0.3;
c= quiver3 (0,0,0,TopModel.Normal(1),TopModel.Normal(2),TopModel.Normal(3),'b'); %model 1
c.AutoScaleFactor = 10;
c.LineWidth = 5;
c.MaxHeadSize = 5;

q= quiver3 (0,0,0,direction(1),direction(2),direction(3),'k'); %cross of normal to model 1 and line
q.AutoScaleFactor = 10;
q.LineWidth = 5;
q.MaxHeadSize = 5;

h= quiver3 (0,0,0,line(1),line(2),line(3),'g'); % line vector
h.AutoScaleFactor = 10;
h.LineWidth = 5;
h.MaxHeadSize = 5;

% plot3 (0,0,0)
fp = fplot3(xt,yt,zt,[-150,50],'gX:');
% scatter3(wrapped(:,1),wrapped(:,2),wrapped(:,3),'b*');
scatter3(mid(1),mid(2),mid(3),'go');





axis auto
hold off
%% 

%% 

figure
pcshow(ramp)
title('two planes and line intersect')%
hold on
%mad = plot(BottomModel, 'color', 'blue');
%mod = plot(RampModel, 'color', 'magenta');
fp = fplot3(xt,yt,zt,[-50,50],'gX:');
scatter3(wrapped(:,1),wrapped(:,2),wrapped(:,3),'b*');
scatter3(mid(1),mid(2),mid(3),'go');
scatter3(centerofprev(1),centerofprev(2),0,'kX');
scatter3(centerofnew(1),centerofnew(2),0,'rX');
scatter3(printcenter(1),printcenter(2),0,'rO')

c= quiver3 (0,0,0,TopModel.Normal(1),TopModel.Normal(2),TopModel.Normal(3),'b'); %model 1
q= quiver3 (0,0,0,direction(1),direction(2),direction(3),'k'); %cross of normal to model 1 and line
h= quiver3 (0,0,0,line(1),line(2),line(3),'g'); % line vector
% plot3 (0,0,0)
  
q.AutoScaleFactor = 10;
q.LineWidth = 5;
q.MaxHeadSize = 5;
c.AutoScaleFactor = 10;
c.LineWidth = 5;
c.MaxHeadSize = 5;
h.AutoScaleFactor = 10;
h.LineWidth = 5;
h.MaxHeadSize = 5;
ax = gca;
ax.Color = 'yellow';
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')

axis auto
hold off
%% 

% figure
% pcshow(ptCloud)
% title('The Point Cloud')
% hold on
% ax = gca;
% ax.Color = 'yellow';
% hold off
% 
% figure
% pcshow(search)
% title('search area')
% hold on
% ax = gca;
% ax.Color = 'yellow';
% hold off

% 
% figure
% pcshow(bottom)
% title('PLatform')
% hold on
% plot(BottomModel)
% plot(RampModel)
% ax = gca;
% ax.Color = 'yellow';
% hold off

% figure
% pcshow(rem1)
% title('!platform')%
% hold on
% ax = gca;
% ax.Color = 'yellow';
% hold off
% 
% figure
% pcshow(top)
% title('top')%
% hold on
% ax = gca;
% ax.Color = 'yellow';
% plot(TopModel)
% hold off
% 
% 
% figure
% pcshow(ramp)
% title('slope')%
% hold on
% ax = gca;
% ax.Color = 'yellow';
% plot(RampModel)
% hold off
% 
% 
% % figure
% % pcshow(ramp)
% % title('super slope')%
% % hold on
% % ax = gca;
% % plot(super3)
% % ax.Color = 'yellow';
% % hold off
% 
% figure
% pcshow(rem1)
% title('before denoise')%
% hold on
% ax = gca;
% ax.Color = 'yellow';
% hold off

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
% plot(TopModel, 'color', 'blue')
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
mad = plot(TopModel, 'color', 'blue');
mod = plot(RampModel, 'color', 'magenta');
fp = fplot3(xt,yt,zt,[-50,50],'r:');
% scatter3(wrapped(:,1),wrapped(:,2),wrapped(:,3),'b*');
zlabel('Z(m)')

axis auto
hold off
alpha(mad,.2)
alpha(mod,.2)



%%%%%%%%%%%%%%%%%%,1),wrapped(:,2),wrapped(:,3),'b*');
% scatter3(midpoint(1),midpoint(2),midpoint(3),'go');
% 
% ax = gca;
% ax.Color = 'yellow';
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% 
% axis auto
% hold off
% alpha(mad,.2)
% alpha(mod,.2)



%%%%%%%%%%%%%%%%%%


%alpha(mad,.2)
%alpha(mod,.2)
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
