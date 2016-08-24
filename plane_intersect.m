% This function is based on 
% plane intersection
% by Nassim Khaled
%  

function [P,N]=plane_intersect(N1,N2)


%get two points A1 and A2 on these two planes
A1 = [0 0 0];
A2 = A1;
A1(3) = -N1.Parameters(4)/N1.Parameters(3);
A2(3) = -N2.Parameters(4)/N2.Parameters(3);

P=[0 0 0]; %point on line of intersection
N=cross(N1.Normal,N2.Normal); %N is the actual vector of the intersection


% Plane 1 and Plane 2 intersect in a line
%first determine max abs coordinate of cross product
maxc=find(abs(N)==max(abs(N)));


%next, to get a point on the intersection line and
%zero the max coord, and solve for the other two
      
d1 = -dot(N1.Normal,A1);   %the constants in the Plane 1 equations
d2 = -dot(N2.Normal, A2);  %the constants in the Plane 2 equations

switch maxc
    case 1                   % intersect with x=0
        P(1)= 0;
        P(2) = (d2*N1.Normal(3) - d1*N2.Normal(3))/ N(1);
        P(3) = (d1*N2.Normal(2) - d2*N1.Normal(2))/ N(1);
    case 2                    %intersect with y=0
        P(1) = (d1*N2.Normal(3) - d2*N1.Normal(3))/ N(2);
        P(2) = 0;
        P(3) = (d2*N1.Normal(1) - d1*N2.Normal(1))/ N(2);
    case 3                    %intersect with z=0
        P(1) = (d2*N1.Normal(2) - d1*N2.Normal(2))/ N(3);
        P(2) = (d1*N2.Normal(1) - d2*N1.Normal(1))/ N(3);
        P(3) = 0;
end

