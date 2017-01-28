% mainGenerateDesiredTrajectory.m   Script for simulating mobile robot's
% reference trajectory

function mainGenerateDesiredTrajectory

% Created by S. Miah on Feb. 01, 2016.
close all
clear all
clc

disp('Please wait while the simulation runs ...');

% ---------------------
% SIMULATION PARAMETERS
% ---------------------

% Simulation time
t0 = 0; tf = 60; % initial and final simulation time [s]
T = 1;  % Sampling time [s]
tsteps = floor((tf-t0)/T); % number of time steps
dt = T*(0:tsteps)'; % Discrete time vector (include time t0 to plot initial state too)
t=dt*T;
% Initial Values
pInit= [3,2,pi/2];
p=pInit;
p_error=[];
p_err_theta=[pi/2];
wkd=-0.1;
vkd=-0.2;
sigma=.02;
    xarray=p(:,1);
    yarray=p(:,2);
    thetaarray=p(:,3);
    vdtarray=vkd;
    wdtarray=wkd;
% Generate reference trajectory (circle) 
for x=1:tf
xdt = p(1) + T*vkd*cos(p(3)+(T*wkd)/2);
ydt = p(2) + T*vkd*sin(p(3)+(T*wkd)/2);
thetadt = p(3) + T*wkd;
while thetadt > pi
    thetadt = thetadt - 2*pi;
end
while thetadt < -pi
     thetadt = thetadt + 2*pi;
end

xd_dott = -T^2*p(:,3)*vkd*T*wkd/2*sin(p(:,3)+(T*wkd)/2);
yd_dott = T^2*p(:,3)*vkd*T*wkd/2*cos(p(:,3)+(T*wkd)/2);

vdt =sqrt(xd_dott.^2 + yd_dott.^2); % [m/s]


xd_dot_dott = -T^3*p(:,3)^2*vkd*(T*wkd/2)^2*cos(p(:,3)+(T*wkd)/2);
yd_dot_dott = -T^3*p(:,3)^2*vkd*(T*wkd/2)^2*sin(p(:,3)+(T*wkd)/2);

tmp = yd_dot_dott.*xd_dott - xd_dot_dott.*yd_dott; 
omegadt = tmp./(xd_dott.^2 + yd_dott.^2);
p= [xdt, ydt, thetadt];
xarray= [xarray, xdt];
yarray=[yarray, ydt];
thetaarray= [thetaarray, thetadt];
vdtarray = [vdtarray, vdt];
wdtarray = [wdtarray, omegadt];

%%ABOVE IS HORRIBLY INEFFICIENT, forgot how to change append an array
%%would have done p(x:3) = [xdt, ydt, thetadt] if having redone it
end
qdInit = [xarray;yarray;thetaarray]';


% Set simulation tolerance (not critical)
RelTol = 1e-3;

% Run the simulation
options = odeset('RelTol',RelTol);

% plot



for x=1:61
d1sq = (qdInit(x,1)-5).^2+(qdInit(x,2)-4).^2+(0-3)^2 + sigma*randn(1,1);
d2sq = (qdInit(x,1)-3).^2+(qdInit(x,2)-8).^2+(0-3)^2 + sigma*randn(1,1);
d3sq = (qdInit(x,1)+3).^2+(qdInit(x,2)-5).^2+(0-3)^2 + sigma*randn(1,1);


p1=[5,4,3]';
p2=[3,8,3]';
p3=[-3,5,3]';

v1 = p2-p1;
v2 = p3-p1;
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

p_error= [p_error p4];

end
p_error= p_error';

%find theta
y=2;
for x=1:60
    
    p_theta= atan2((p_error(x,2)-p_error(y,2)),(p_error(x,1)-p_error(y,1)));
    y= y+1;
    p_err_theta = [p_err_theta p_theta];
    
    while p_theta > pi
        p_theta = p_theta - 2*pi
    end
    while p_theta < -pi
         p_theta = p_theta + 2*pi
    end

end
 p_err_theta=  p_err_theta';
true_p_error = [p_error(:,1) p_error(:,2) p_err_theta];

%%
    figure(2)
    subplot(3,1,1)
        plot(dt,qdInit(:,1))    
        hold on
        d=plot(dt,true_p_error(:,1))
        set(d, 'LineWidth', 2 ,{'LineStyle'},{'--'},{'Color'},{'b'});
        title('x(t)')
        legend('True Pose X','Estimated Pose X')
        ylabel('Distance (m)') % x-axis label
        xlabel('Time (s)') % y-axis label
    subplot(3,1,2)
        plot(dt,qdInit(:,2))
        hold on
        e=plot(dt,true_p_error(:,2))
        set(e, 'LineWidth', 2 ,{'LineStyle'},{'--'},{'Color'},{'b'});
        title('y(t)')
        legend('True Pose Y','Estimated Pose Y')
        ylabel('Distance (m)') % x-axis label
        xlabel('Time (s)') % y-axis label
    subplot(3,1,3)
        plot(dt,qdInit(:,3))
        hold on
        f=plot(dt,true_p_error(:,3))
        set(f, 'LineWidth', 2 ,{'LineStyle'},{'--'},{'Color'},{'b'});
        title('Theta(t)')
        legend('True Pose Theta','Estimated Pose Theta')
        ylabel('Distance (m)') % x-axis label
        xlabel('Time (s)') % y-axis label
%%initialize values
x_err0 = [];
y_err0 = [];
theta_err0 = [];
x_err = [];
y_err = [];
theta_err = [];P1_1=[];P2_2=[];P3_3=[];
%%
%%generate robots state error propagation

    for x=1:61
    p_0= diag((qdInit(x,:)-true_p_error(x,:)).^2);
    x_err0 = [x_err0 p_0(1,1)];
    y_err0 = [y_err0 p_0(2,2)];
    theta_err0 = [theta_err0 p_0(3,3)];
    end
    
    %%ERROR PROPAGATION FUNCTION
    %Derive these values    
    %xdt = p(1) + T*vkd*cos(p(3)+(T*wkd)/2);
    %ydt = p(2) + T*vkd*sin(p(3)+(T*wkd)/2);
    %thetadt = p(3) + T*wkd;
    for x=1:61
    Fk = [1 0 -T*vkd*sin(qdInit(3)+(T*wkd)/2); 0 1 T*vkd*cos(qdInit(3)+(T*wkd)/2) ; 0 0 1];
    Pk= (Fk)*(p_0)*(Fk)';
    P1_1=[P1_1 Pk(1,1)];
    P2_2=[P2_2 Pk(2,2)];
	P3_3=[P3_3 Pk(2,2)];
    end
    %% 
    
    for x=1:61
    p_v= diag((qdInit(x,:)-true_p_error(x,:)));
    x_err = [x_err p_v(1,1)];
    y_err = [y_err p_v(2,2)];
    theta_err = [theta_err p_v(3,3)];
    end
    x_err0(1:61)=var(x_err0) ;
    y_err0(1:61)=var(y_err0) ;
    theta_err0(1:61)=var(theta_err0) ;
    
    %% PLOT ERROR with Bounds
    figure(5)
    subplot(3,1,1)
        plot(x_err)
        hold on
        %since P(1,1) is sigma, sigma =0.02
        
        %%plot(x_err+3*.02*sqrt(.02))
        %%plot(x_err-3*.02*sqrt(.02))
        %%IS ABOVE RIGHT?
        plot(x_err-3*sqrt(x_err0))
        hold on
        plot(x_err+3*sqrt(x_err0))
        title('Pose Error of X vs Time')
        ylabel('Distance (m)') % x-axis label
        xlabel('Time (s)') % y-axis label
    subplot(3,1,2)
        plot(y_err)
        hold on
        %plot(y_err+3*sqrt(var(P2_2)))
        %%SHOULD BE VAR(P2_2) but isn't it too low?
        plot(y_err+3*sqrt(y_err0))
        hold on
        %plot(y_err-3*sqrt(var(P2_2)))
        plot(y_err-3*sqrt(y_err0))
        title('Pose Error of Y vs Time')
        ylabel('Distance (m)') % x-axis label
        xlabel('Time (s)') % y-axis label
    subplot(3,1,3)
    plot(theta_err)
        hold on
        plot(theta_err+3*sqrt(theta_err0))
        hold on
        plot(theta_err-3*sqrt(theta_err0))
        title('Pose Error of Theta vs Time')
        ylabel('Distance (m)') % x-axis label
        xlabel('Time (s)') % y-axis label
    


disp('... done.');

