syms t;


xt = t;
yt = t;
zt = 0*t;

x = linspace(1,30,30);
x= x(:);
y = x;
z = zeros(30,1);

[sega,segb,segc] = wherestheline(t,xt,yt,zt,x,y,z);