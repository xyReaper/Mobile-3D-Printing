%version 1.0
function [wrapped] = wherestheline(t,xt,yt,zt,x,y,z)

tic;

%  syms t;
%  xt = t;
%  yt = t;
%  zt = 0*t;
%  
%  x = linspace(1,30,30);
%  x= x(:);
%  y = x;
%  z = zeros(30,1);
% 
[m,n] = size(x);
sega = zeros(m,n);
segb = zeros(m,n);
segc = zeros(m,n);


situationx = 1;
%version 1.0
situationy = 1;
situationz = 1;

%%%check what case%%%%
if (xt() == 0)
    situationx = 0;
end

if (yt() == 0)
    situationy = 0;
end

if (zt() == 0)
    situationz = 0;
end
%%%%%%%%%%

%%%do the mega check%%%%%
if ((situationx == 0)&& (situationy == 0)&&(situationz == 0))
    warning('all are zero functions');
    situation = 0;
elseif ((situationx == situationy) && (situationy == 0))
    situation = 1;
elseif ((situationx == situationz) && (situationz == 0))
    situation = 2;
elseif ((situationy == situationz) &&(situationz == 0))
    situation = 4;
elseif (situationx == 0)
    situation = 3;
elseif (situationy == 0)
    situation = 5;
elseif (situationz == 0)
    situation = 6;
else
    situation = 7;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%Choose wehre to go%%%%%%%
%%%X,Y,Z Case
%%%0,0,0 0 
%%%0,0,1 1
%%%0,1,0 2
%%%0,1,1 3
%%%1,0,0 4
%%%1,0,1 5
%%%1,1,0 6
%%%1,1,1 7

switch situation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    case  7 
        parfor i = 1:m
            testx = solve(xt == x(i),t,'real',true);
            testy = solve(yt == y(i),t,'real',true);
            testz = solve(zt == z(i),t,'real',true);
            if aretheyequal(testx,testy) == 1
                if aretheyequal(testy,testz) == 1
                    sega(i) = x(i);
                    segb(i) = y(i);
                    segc(i) = z(i);
                end
            end
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 3 
        parfor i = 1:m
            if x(i) == 0
                testy = solve(yt == y(i),t,'real',true);
                testz = solve(zt == z(i),t,'real',true);
                if aretheyequal(testy,testz) == 1
                        sega(i) = 0;
                        segb(i) = y(i);
                        segc(i) = z(i);
                end
            end
        end
           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
   
    case 5 
        parfor i = 1:m
            if y(i) == 0
                testx = solve(xt == x(i),t,'real',true);
                testz = solve(zt == z(i),t,'real',true);
                if aretheyequal(testx,testz) == 1
                        sega(i) = x(i);
                        segb(i) = 0;
                        segc(i) = z(i);
                end
            end
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 6 
        parfor i = 1:m
            if z(i) == 0
                testx = solve(xt == x(i),t,'real',true);
                testy = solve(yt == y(i),t,'real',true);
                if aretheyequal(testx,testy) == 1
                        sega(i) = x(i);
                        segb(i) = y(i);
                        segc(i) = 0;
                end
            end
        end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 1 
        parfor i = 1:m
            if x(i) == y(i) == 0
                    sega(i) = 0;
                    segb(i) = 0;
                    segc(i) = z(i);
            end
        end
        
     case 2 
        parfor i = 1:m
            if x(i) == z(i) == 0
                    sega(i) = 0;
                    segb(i) = z(i);
                    segc(i) = 0;
            end
        end
        
     case 4 
        parfor i = 1:m
            if y(i) == z(i) == 0
                    sega(i) = x(i);
                    segb(i) = 0;
                    segc(i) = 0;
            end
        end        
        
end


wrapped = [sega segb segc];
wrapped( all(~wrapped,2), :) = [];
% 
% figure
% title('two planes and line intersect')%
% 
% fp = fplot3(xt,yt,zt,[-50,50],'r--o');
% hold on
% ax = gca;
% ax.Color = 'yellow';
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% axis equal
% axis auto
% zoom off
% hold off


TimeSpent=toc;
end
