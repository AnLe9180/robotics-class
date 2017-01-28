%%-----------------------------------------------------------------------%%
%This example code will create a ROS node and then subscribe to the
%position topic of the Pioneer 3DX. It will then create a publisher to the
%Pioneer 3DX's velocity topic and position setting topic. For this to work
%first make sure you are connected to ECE-Robotics1 Wifi, there is no
%password for this WiFi but there is also no internet so keep that in mind.
%
%This simple test program will connect to the Pioneer 3DX and drive it
%forward for 3 seconds while recording the position data from the robot.
%Then stop the robot and finish.

%To use this example type rosinit() in the command window. Then open Vrep
%and load the scene PioneerSimulation.ttt. Then start the simulation. Once
%the simulation is started run simulationInterface in another instance of
%Matlab. Wait for that program to say Initalized. Then run this code and
%watch the robot move!

%When done simulating press control+c in the command windows of the matlab
%window running the simulatorInterface. Then vrep can be easily closed
%after that.

%simulationInterface is for 32 bit matlab
%simulationInterface64 is for 64 bit matlab
%Open the correct version that matches your bit version of matlab
%%
function mainExample
    clc;
    clear;
    close all;
    pause('on');
%%  

%
    T = 0.6; % [s]
    t0 = 0;
    tf = 60; % [s]
    tsteps = floor((tf-t0)/T);
    dt = T*(0:tsteps)';
    bot = zeros(length(dt),4);
    
gamma = 2*pi/(tf-t0);
R = 1; % radius of the circle [m]
center = [1.5 1.5]; % [m]
    
    nodeName = 'BlackBeard'; %Change to your name
    rosCoreIP = '192.168.1.3'; %This is the IP address of the core object for ros. This is running on one of the robots.    
    %nodeName = 'AlexFerrebee'; %Change to your name
    %rosCoreIP = 'localhost'; %This is the IP address of the core object for ros. This is running on one of the robots.
    robotNodeName = '/Pioneer3dx';  %This is the name of the node for the Pioneer 3 DX
    node = robotics.ros.Node(nodeName,rosCoreIP);   %This creates a node with your name that connects to the ROS Core
    cmd_Pub = robotics.ros.Publisher(node,strcat(robotNodeName,'/cmd_vel'),'geometry_msgs/Twist'); %This creates a ROS Publisher that will write to the velocity topic of the robot
    setPose_Pub = robotics.ros.Publisher(node,strcat(robotNodeName,'/setPose'),'nav_msgs/Odometry'); %This creates a ROS Publisher that will write to the Position setting topic of the robot
    pose_Sub = robotics.ros.Subscriber(node,strcat(robotNodeName,'/pose')); %This creates a ROS Subscriber that will read the pose of the robot

    pose = rosmessage('nav_msgs/Odometry'); %This creates a message object that stores the pose of the robot
    velocity = rosmessage('geometry_msgs/Twist'); %This creates a message object that stores the velocites of the robot

    pose.Pose.Pose.Position.X = 2;          %[meters]
    pose.Pose.Pose.Position.Y = 1;          %[meters]
    pose.Pose.Pose.Orientation.W = pi/2;    %[Radians]
    send(setPose_Pub,pose);    %This sends the position to the robot so that the internal odometry is set
    pause(2);
    cleanupObj = onCleanup(@cleanMeUp);
    disp('Initalized');
%%    
    for k = 0:length(dt)
    t = k*T;
    xdt = center(1) + R*cos(gamma*t);  
    ydt = center(2) + R*sin(gamma*t);
    
    xd_dott = -R*gamma*sin(gamma*t);
    yd_dott = R*gamma*cos(gamma*t);
    thetadt = atan2(yd_dott, xd_dott);
    
    xd_dot_dott = -R*gamma^2*cos(gamma*t);
    yd_dot_dott = -R*gamma^2*sin(gamma*t);
    
    tmp = yd_dot_dott*xd_dott - xd_dot_dott*yd_dott;
    omegadt = tmp/(xd_dott^2 + yd_dott^2);
    
    vkd =sqrt(xd_dott^2 + yd_dott^2);
        pause(T);   %Pause for T seconds
        
    velocity.Linear.X = vkd;    %Forward Velocity [m/sec]
    velocity.Angular.Z = omegadt;   %Angular Velocity [rad/sec]
    send(cmd_Pub,velocity);     %Send both Forward and Angular velocity to robot
    end
    velocity.Linear.X = 0; 
    velocity.Angular.Z = 0;   %Angular Velocity [rad/sec]
    send(cmd_Pub,velocity);     %Send both Forward and Angular velocity to robot
end

%%This simple function will be run at the exit of the program even if the
%%cause of the exit is an error. It will preform and tasks you put inside
%%the function definition.

function cleanMeUp()
    clear node;
    clear cmd_Pub;
    clear setPose_Pub;
    clear pose_Sub;
    disp('Cleaned up');
end