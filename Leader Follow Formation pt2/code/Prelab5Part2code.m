function Prelab5Part2code
% Created by J. Morris on Apr. 12, 2016.
close all
clear all
clc

disp('Please wait while the simulation runs ...');

% ---------------------
% SIMULATION PARAMETERS
% ---------------------

% Simulation time
t0 = 0; tf = 94; % initial and final simulation time [s]
T = 0.1;  % Sampling time [s]
tsteps = floor((tf-t0)/T); % number of time steps
dt = T*(0:tsteps)'; % Discrete time vector (include time t0 to plot initial state too)
gamma = (1/15);
zeta = .7;
b = 1;
a = 1;

Tau = 5;
Tausteps = floor(Tau/T);
Tau = T*(0:Tausteps)';

qInit = [1, 1, (pi/2)];
% qInit = [1.5, 1.5, .464];
qkprnew = qInit;

[desiredPose, vddt, omegaddt] = Prelab5Part1();

    numit = length(desiredPose(:, 1));
    
    vddt(numit, :) = vddt(numit-1, :);
    omegaddt(numit, :) = omegaddt(numit-1, :);
    
for i = 1:(Tausteps)
desiredPose(numit+i,:) = desiredPose(numit,:);
vddt(numit+i,:) = vddt(numit,:);
omegaddt(numit+i,:) = omegaddt(numit,:);
end

xdes = desiredPose(:, 1);
ydes = desiredPose(:, 2);
thetades = desiredPose(:, 3);

% k1 = (2*zeta*a);
k1 = (6);
k2 = b;
k3 = k1;

% disp((tsteps+Tausteps+1))
% disp(length(vddt))

for k = 0:tsteps+Tausteps+1
    
    t = k*T;
    dtnew(k+1,1) = t;
    actualPose(k+1, :) = qkprnew;
    
    if(k >= Tausteps)
        kpr = k - Tausteps + 1;
        
        e_xpr = ((xdes(kpr, 1)-(actualPose(k+1,1)))*cos(actualPose(k+1, 3)))+((ydes(kpr, 1)-(actualPose(k+1,2)))*sin(actualPose(k+1, 3)));
        e_ypr = (((-(xdes(kpr, 1)))-actualPose(k+1,1))*sin(actualPose(k+1, 3)))+((ydes(kpr, 1)-actualPose(k+1,2))*cos(actualPose(k+1, 3)));
        e_thetapr = thetades(kpr, 1) - actualPose(k+1,3);
        
        u1pr = (-k1*e_xpr);
        u2pr = (-k2*vddt(k+1, 1)*(((sin(e_thetapr)))/(e_thetapr))*e_ypr)-(k3*e_thetapr);
        
        vkpr(k+1, 1) = (vddt(kpr, 1)*cos(e_thetapr))-u1pr;
        omegadtpr(k+1, 1) = omegaddt(kpr, 1) - u2pr;
        
        delta_thetapr = (T*omegadtpr(k+1,1));
        
        xkprnew = actualPose(k+1, 1)+(T*vkpr(k+1,1)*cos(actualPose(k+1, 3)+(delta_thetapr/2)));
        ykprnew = actualPose(k+1, 2)+(T*vkpr(k+1,1)*sin(actualPose(k+1, 3)+(delta_thetapr/2)));

        thetakprnew = actualPose(k+1, 3) + delta_thetapr;
        
        if(thetakprnew>pi)
        thetakprnew = thetakprnew - (2*pi);
        end
    
        if(thetakprnew<-pi)
        thetakprnew = thetakprnew + (2*pi);
        end
        
        qkprnew = [xkprnew,ykprnew,thetakprnew];
%         actualPose(k+2, :) = qkprnew;
        
    end
end


% for i = 1:3
% newDesired(:,i) = desiredPose(:,i);
% end

poseError = desiredPose-actualPose;


generatePlots(length(dtnew), dtnew, actualPose, desiredPose, poseError, [vkpr omegadtpr]);
% generatePlots(length(dtnew), dtnew, [], actualPose, [], [vddt omegaddt]);


disp('... done.');


function edot = stateEqError(t,e,k1,k2,k3,dt,vdt,omegadt)

omegad = interp1(dt, omegadt, t);
k2t = interp1(dt, k2, t);
vd = interp1(dt, vdt, t);

At=[-k1 omegad 0;
    -omegad 0 vd;
    0 -k2t -k3];

edot = At*e;


% Reference robot kinematic model
function qddot = stateEqDesired(t,qd,dt,vdt,omegadt)

vd = interp1(dt,vdt,t);
omegad = interp1(dt,omegadt,t);
thetad = qd(3);
xddott = vd*cos(thetad);
yddott = vd*sin(thetad);
thetaddott = omegad;

qddot = [xddott;
         yddott;
         thetaddott];



function generatePlots(Tn, t, actualStates, desiredStates, error, u)    
    close all; % close all opened figures
    % Tn = number of discrete time/path parameter points 
    % t = time/path parameter history, dimension = Tn x 1 
    % actualState = actual state of the system, dimension = Tn x n
    % desiredState = desired state of the system, dimension = Tn x n
    % error = desired state - actual state, dimension =  Tn x n
    % u = control inputs, dimension = Tn x m
         
 
    % Plot the robot's  velocities, 
    figure
    subplot(2,1,1)
    plot(t,u(:,1), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('Linear speed [m/s]');        
    grid on
    
    subplot(2,1,2)
    plot(t,u(:,2), 'k--','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('Angular speed [rad/s]');
    grid on
    
    savefilename = ['OUT/controlInputs'];
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
    
    % Create a movie of the simulation (path/trajectory)
    

    xmax = max(desiredStates(:,1));
    xmin = min(desiredStates(:,1));
    ymax = max(desiredStates(:,2));
    ymin = min(desiredStates(:,2));
          
    vid = VideoWriter('OUT/trajectory.avi');
    vid.Quality = 100;
    vid.FrameRate = 5;
    open(vid)
    
    fig=figure;
    clf reset;    
    
    for i = 1:Tn,
        clf;
        box on;
        axis([xmin-5 xmax+5 ymin-5 ymax+5]);
        axis equal;
        axis manual;
        [Xa,Ya] = plot_DDMR(actualStates(i,:),axis(gca)); % 
        hold on;
        desired = plot(desiredStates(1:i,1),desiredStates(1:i,2),'LineWidth',1.5);
        hold on
        actual = plot(actualStates(1:i,1),actualStates(1:i,2),'k--');
        fill(Xa,Ya,'r');
        hold off;
        xlabel('x [m]');
        ylabel('y [m]');
        F = getframe(fig);
        writeVideo(vid,F);          
    end
    [Xa,Ya] = plot_DDMR(actualStates(1,:),axis(gca)); % DDMR => Differential drive mobile robot
    grid on
    hold on
    plot(Xa,Ya);    
    hold on
    %legend([actual desired],'actual', 'desired');
    savefilename = 'OUT/trajectory';
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
       
    close(vid);

    % Create the movie and save it as an AVI file
    % winopen('OUT/trajectory.avi')

    
    

function [X,Y] = plot_DDMR(Q,AX)
% PLOT_UNICYCLE   Function that generates lines for plotting a unicycle.
%
%    PLOT_UNICYCLE(Q,AX) takes in the axis AX = axis(gca) and the unicycle
%    configuration Q and outputs lines (X,Y) for use, for example, as:
%       fill(X,Y,'b')
%    This must be done in the parent script file.

x     = Q(1);
y     = Q(2);
theta = Q(3);

l1 = 0.02*max([AX(2)-AX(1),AX(4)-AX(3)]);
X = [x,x+l1*cos(theta-2*pi/3),x+l1*cos(theta),x+l1*cos(theta+2*pi/3),x];
Y = [y,y+l1*sin(theta-2*pi/3),y+l1*sin(theta),y+l1*sin(theta+2*pi/3),y];