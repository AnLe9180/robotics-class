%Code written by Jason Morris, Mike McGrath, and Anthony Le

clear all
close all
clc

fname = 'posePart1.txt';
fileop = fopen(fname, 'r');
data = dlmread(fname, ' ');


figure(1);
subplot(3,1,1);
    plot(data(:,1), data(:,2), 'k');
    title('x, y, and theta pose');
    xlabel('time(s)');
    ylabel('x(m)');
    grid on;
subplot(3,1,2);
    plot(data(:,1), data(:,3), 'k');
    xlabel('time(s)');
    ylabel('y(m)');
    grid on;
subplot(3,1,3);
    plot(data(:,1), data(:,4), 'k');
    xlabel('time(s)');
    ylabel('\theta(rad)');
    grid on;


figure(2);
subplot(2,1,1);
    plot(data(:,1), data(:,5), 'k');
    title('Linear and Angular Velocities');
    xlabel('time(s)');
    ylabel('Linear Velocity(m/s)');
    grid on;
subplot(2,1,2);
    plot(data(:,1), data(:,6), 'k');
    xlabel('time(s)');
    ylabel('Angular Velocity(rad/s)');
    grid on;
    
fclose(fileop);