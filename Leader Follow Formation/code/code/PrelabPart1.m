function desired = PrelabPart1new
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
T = 0.6;  % Sampling time [s]
tsteps = floor((tf-t0)/T); % number of time steps
dt = T*(0:tsteps)'; % Discrete time vector (include time t0 to plot initial state too)
gamma = 2*pi/(tf-t0);


% Generate reference trajectory (circle) 
R = 1; % radius of the circle [m]
center = [1.5 1.5]; % [m]

for k = 0:1:tsteps
    
    t = k*T;
    xdt = center(1) + R*cos(gamma*t);  
    ydt = center(2) + R*sin(gamma*t);
    
    xd_dott = -R*gamma*sin(gamma*t);
    yd_dott = R*gamma*cos(gamma*t);
    thetadt = atan2(yd_dott, xd_dott);
    
    xd_dot_dott = -R*gamma^2*cos(gamma*t);
    yd_dot_dott = -R*gamma^2*sin(gamma*t);
    
    tmp = yd_dot_dott.*xd_dott - xd_dot_dott.*yd_dott;
    omegadt(k+1, 1) = tmp./(xd_dott.^2 + yd_dott.^2);
    
    vkd(k+1, 1) =sqrt(xd_dott.^2 + yd_dott.^2);
    
    qk = [xdt,ydt,thetadt];
    
        if k == 0
        desiredPose = qk;
        end
        
    desiredPose(k+1, :) = qk;

    delta_theta = T*omegadt(1,1);
    xdtnew = xdt + T*vkd(1,1)*cos(thetadt + (delta_theta/2));
    ydtnew = ydt + T*vkd(1,1)*sin(thetadt + (delta_theta/2));
    thetadtnew = thetadt + delta_theta;
    qknew = [xdtnew,ydtnew,thetadtnew];
    
%     desiredPose(k+2, :) = qknew;
    
end

desired = [desiredPose omegadt vkd];


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
        [Xa,Ya] = plot_DDMR(desiredStates(i,:),axis(gca)); % 
        hold on;
        desired = plot(desiredStates(1:i,1),desiredStates(1:i,2),'LineWidth',1.5);
        hold on
        %actual = plot(actualStates(1:i,1),actualStates(1:i,2),'k--');
        fill(Xa,Ya,'r');
        hold off;
        xlabel('x [m]');
        ylabel('y [m]');
        F = getframe(fig);
        writeVideo(vid,F);          
    end
    [Xa,Ya] = plot_DDMR(desiredStates(1,:),axis(gca)); % DDMR => Differential drive mobile robot
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


