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
    T = 0.2; % [s]
    t0 = 0;
    tf = 60; % [s]
    tsteps = floor((tf-t0)/T);
    dt = T*(0:tsteps)';
    bot = zeros(length(dt),4);
    
gamma = 2*pi/(tf-t0);
R = 1; % radius of the circle [m]
center = [1.5 1.5]; % [m]

Tau = 3;
Tausteps = (Tau/T);
Tau = T*(0:Tausteps)';

qInit = [2, 1, (pi/2)];
qkprnew = qInit;

% Generate reference trajectory (circle) 
R = 1; % radius of the circle [m]
center = [1.5 1.5]; % [m]

zeta = 0.5;
a = 0.2;

    
    nodeName = 'Powerfalcon'; %Change to your name
    rosCoreIP = '192.168.1.3'; %This is the IP address of the core object for ros. This is running on one of the robots.    
    %nodeName = 'AlexFerrebee'; %Change to your name
    %rosCoreIP = 'localhost'; %This is the IP address of the core object for ros. This is running on one of the robots.
    Leader_robotNodeName = '/Pioneer3dx';  %This is the name of the node for the Pioneer 3 DX
    node = robotics.ros.Node(nodeName,rosCoreIP);   %This creates a node with your name that connects to the ROS Core
    Leader_cmd_Pub = robotics.ros.Publisher(node,strcat(Leader_robotNodeName,'/cmd_vel'),'geometry_msgs/Twist'); %This creates a ROS Publisher that will write to the velocity topic of the robot
    Leader_setPose_Pub = robotics.ros.Publisher(node,strcat(Leader_robotNodeName,'/setpose'),'nav_msgs/Odometry'); %This creates a ROS Publisher that will write to the Position setting topic of the robot
    Leader_pose_Sub = robotics.ros.Subscriber(node,strcat(Leader_robotNodeName,'/pose')); %This creates a ROS Subscriber that will read the pose of the robot

    Leader_pose = rosmessage('nav_msgs/Odometry'); %This creates a message object that stores the pose of the robot
    Leader_velocity = rosmessage('geometry_msgs/Twist'); %This creates a message object that stores the velocites of the robot

    Leader_pose.Pose.Pose.Position.X = 2.5;          %[meters]
    Leader_pose.Pose.Pose.Position.Y = 1.5;          %[meters]
    Leader_pose.Pose.Pose.Orientation.W = pi/2;    %[Radians]
    send(Leader_setPose_Pub,Leader_pose);    %This sends the position to the robot so that the internal odometry is set
    pause(2);
    disp('Initalized');
%%  
    %nodeName = 'AlexFerrebee'; %Change to your name
    %rosCoreIP = 'localhost'; %This is the IP address of the core object for ros. This is running on one of the robots.
    Follow_robotNodeName = '/Pioneer2';  %This is the name of the node for the Pioneer 3 DX
    Follow_setPose_Pub = robotics.ros.Publisher(node,strcat(Follow_robotNodeName,'/setPose'),'nav_msgs/Odometry'); %This creates a ROS Publisher that will write to the Position setting topic of the robot
    Follow_pose_Sub = robotics.ros.Subscriber(node,strcat(Follow_robotNodeName,'/pose')); %This creates a ROS Subscriber that will read the pose of the robot

    Follow_pose = rosmessage('nav_msgs/Odometry'); %This creates a message object that stores the pose of the robot
    Follow_velocity = rosmessage('geometry_msgs/Twist'); %This creates a message object that stores the velocites of the robot

    Follow_pose.Pose.Pose.Position.X = 2;          %[meters]
    Follow_pose.Pose.Pose.Position.Y = 1;          %[meters]
    Follow_pose.Pose.Pose.Orientation.W = pi/2;    %[Radians]
    send(Follow_setPose_Pub,Leader_pose);    %This sends the position to the robot so that the internal odometry is set
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
        
    Leader_velocity.Linear.X = vkd;    %Forward Velocity [m/sec]
    Leader_velocity.Angular.Z = omegadt;   %Angular Velocity [rad/sec]
    send(Leader_cmd_Pub,Leader_velocity);     %Send both Forward and Angular velocity to robot
    
    Leader_pose = receive(Leader_pose_Sub);
 for k = 0:tsteps+Tausteps
    dtnew = t;
    
    
    if(k >= Tausteps)
        Follow_pose_Sub = robotics.ros.Subscriber(node,strcat(Follow_robotNodeName,'/pose')); %This creates a ROS Subscriber that will read the pose of the robot
        kpr = k - Tausteps + 1;
        %Follow_pose.Pose.Pose.Position.X Follow_pose.Pose.Pose.Orientation.W
        e_xpr = (Leader_pose.Pose.Pose.Position.X-Follow_pose.Pose.Pose.Position.X)*cos(Follow_pose.Pose.Pose.Orientation.W)+(Leader_pose.Pose.Pose.Position.Y-Follow_pose.Pose.Pose.Position.X)*sin(Follow_pose.Pose.Pose.Orientation.W);
        e_ypr = -(Leader_pose.Pose.Pose.Position.X-Follow_pose.Pose.Pose.Position.X)*sin(Follow_pose.Pose.Pose.Orientation.W)+(Leader_pose.Pose.Pose.Position.Y-Follow_pose.Pose.Pose.Position.X)*cos(Follow_pose.Pose.Pose.Orientation.W);
        e_thetapr = Leader_pose.Pose.Pose.Orientation.W - Follow_pose.Pose.Pose.Orientation.W;
        

        k1 = 1.7;
        k2 = ((a^2) - ((Leader_velocity.Linear.X).^2))/(abs(Leader_velocity.Angular.Z));
        k3 = k1;
        
        u1pr = -k1*e_xpr;
        u2pr = -k2*e_ypr - k3*e_thetapr;
        
        vkdpr = Leader_velocity.Linear.X*cos(e_thetapr)-u1pr;
        omegadtpr = Leader_velocity.Linear.X - u2pr;
        Follow_velocity.Linear.X = vkdpr;    %Forward Velocity [m/sec]
   	    Follow_velocity.Angular.Z = omegadtpr;   %Angular Velocity [rad/sec]    
        send(Follow_cmd_Pub,Follow_velocity);     %Send both Forward and Angular velocity to robot
        receive(Follow_setPose_Pub,Follow_pose);
       
    end
     continue;
 end
    end
    
   
    Leader_velocity.Linear.X = 0; 
    Leader_velocity.Angular.Z = 0;   %Angular Velocity [rad/sec]
    send(Leader_cmd_Pub,Leader_velocity);     %Send both Forward and Angular velocity to robot
    Follow_velocity.Linear.X = 0; 
    Follow_velocity.Angular.Z = 0;   %Angular Velocity [rad/sec]
    send(Follow_cmd_Pub,Follow_velocity);     %Send both Forward and Angular velocity to robot
end

%%This simple function will be run at the exit of the program even if the
%%cause of the exit is an error. It will preform and tasks you put inside
%%the function definition.

function cleanMeUp()
    clear all;
    clear Leader_pose_Sub;
    disp('Cleaned up');
end




















