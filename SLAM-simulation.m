%finding pose using beacons
%Anthony Le

%%non-noise signals
%%
d1 = 4.6904;
d2 = 7.6158;
d3 = 7.8102;
syms x y z
% d1^2 = 22, d2^2= 58, d3^2 = 61
d1sq=d1^2;
d2sq=d2^2;
d3sq=d3^2;
[solx, soly, solz] = solve((x-5)^2+(y-4)^2+(z-3)^2 == d1sq,(x-3)^2+(y-8)^2+(z-3)^2==d2sq, ...
    (x+3)^2+(y-5)^2+(z-3)^2 == d3sq);

solx = double(solx);
soly = double(soly);
solz = double(solz);
x = solx(2,1);
y = soly(2,1);
z = solz(2,1);
%test for answer

p1=[5,4,3]';
p2=[3,8,3]';
p3=[-3,5,3]';

v1 = p2-p1;
v2 = p3-p1
vcross= cross(v1,v2);
Dp12=[norm(p1-p2)]^2;
Dp13=[norm(p1-p3)]^2;
Dp21=[norm(p2-p1)]^2;
Dp23=[norm(p2-p3)]^2;
Dp31=[norm(p3-p1)]^2;
Dp32=[norm(p3-p2)]^2;


Dp123=2*(-1/2)^3*det([0 1 1 1;1 0 Dp12 Dp13;1 Dp21 0 Dp23;1 Dp31 Dp32 0]);

Dp1234=2*(-1/2)^4*det([0 1 1 1 1;1 0 Dp12 Dp13 d1sq;1 Dp21 0 Dp23 d2sq;1 Dp31 Dp32 0 d3sq; 1 d1sq d2sq d3sq 0]);

Dp123p134=2*(-1/2)^3*det([0 1 1 1; 1 0 Dp13 d1sq; 1 Dp21 Dp23 d2sq; 1 Dp31 0 d3sq]);

Dp123p124=2*(-1/2)^3*det([0 1 1 1; 1 0 Dp12 d1sq; 1 Dp21 0 d2sq; 1 Dp31 Dp32 d3sq]);


p4 = p1 + (1/Dp123) * [-Dp123p134*v1 + Dp123p124*v2 - sqrt(Dp1234)*cross(v1,v2)];
fprintf('The robots position with non-noisy signals are (x,y,z, respectively): \n');
disp(p4);


%%with noise
%%
sigma = 0.02;
nd1 = 4.6904 + sigma*randn(1,1);
nd2 = 7.6158 + sigma*randn(1,1);
nd3 = 7.8102 + sigma*randn(1,1);
syms x y z
% d1^2 = 22, d2^2= 58, d3^2 = 61
nd1sq=nd1^2;
nd2sq=nd2^2;
nd3sq=nd3^2;
[solx, soly, solz] = solve((x-5)^2+(y-4)^2+(z-3)^2 == nd1sq,(x-3)^2+(y-8)^2+(z-3)^2==nd2sq, ...
    (x+3)^2+(y-5)^2+(z-3)^2 == nd3sq);

solx = double(solx);
soly = double(soly);
solz = double(solz);
x = solx(2,1);
y = soly(2,1);
z = solz(2,1);

p1=[5,4,3]';
p2=[3,8,3]';
p3=[-3,5,3]';

v1 = p2-p1; v2 = p3-p1; 

Dp12=[norm(p1-p2)]^2;
Dp13=[norm(p1-p3)]^2;
Dp21=[norm(p2-p1)]^2;
Dp23=[norm(p2-p3)]^2;
Dp31=[norm(p3-p1)]^2;
Dp32=[norm(p3-p2)]^2;


Dp123=2*(-1/2)^3*det([0 1 1 1;1 0 Dp12 Dp13;1 Dp21 0 Dp23;1 Dp31 Dp32 0]);

Dp1234=2*(-1/2)^4*det([0 1 1 1 1;1 0 Dp12 Dp13 nd1sq;1 Dp21 0 Dp23 nd2sq;1 Dp31 Dp32 0 nd3sq; 1 nd1sq nd2sq nd3sq 0]);

Dp123p134=2*(-1/2)^3*det([0 1 1 1; 1 0 Dp13 nd1sq; 1 Dp21 Dp23 nd2sq; 1 Dp31 0 nd3sq]);

Dp123p124=2*(-1/2)^3*det([0 1 1 1; 1 0 Dp12 nd1sq; 1 Dp21 0 nd2sq; 1 Dp31 Dp32 nd3sq]);


np4 = p1 + (1/Dp123) * [-Dp123p134*v1 + Dp123p124*v2 - sqrt(Dp1234)*cross(v1,v2)];
fprintf('The robots position with noisy signals are (x,y,z, respectively): \n');
disp(np4);