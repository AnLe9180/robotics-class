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

Tau = 5;
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
    Leader_robotNodeName = '/Pioneer2';  %This is the name of the node for the Pioneer 3 DX
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
    Follow_robotNodeName = '/Pioneer3dx';  %This is the name of the node for the Pioneer 3 DX
    Follow_cmd_Pub = robotics.ros.Publisher(node,strcat(Follow_robotNodeName,'/cmd_vel'),'geometry_msgs/Twist'); 
    Follow_setPose_Pub = robotics.ros.Publisher(node,strcat(Follow_robotNodeName,'/setpose'),'nav_msgs/Odometry'); %This creates a ROS Publisher that will write to the Position setting topic of the robot
    Follow_pose_Sub = robotics.ros.Subscriber(node,strcat(Follow_robotNodeName,'/pose')); %This creates a ROS Subscriber that will read the pose of the robot

    
    Follow_pose = rosmessage('nav_msgs/Odometry'); %This creates a message object that stores the pose of the robot
    Follow_velocity = rosmessage('geometry_msgs/Twist'); %This creates a message object that stores the velocites of the robot

    Follow_pose.Pose.Pose.Position.X = 2;          %[meters]
    Follow_pose.Pose.Pose.Position.Y = 1;          %[meters]
    Follow_pose.Pose.Pose.Orientation.W = pi/2;    %[Radians]
    send(Follow_setPose_Pub,Follow_pose);    %This sends the position to the robot so that the internal odometry is set
    pause(2);
    cleanupObj = onCleanup(@cleanMeUp);
    disp('Initalized');
   
    
%%


    for k = 0:length(dt+Tau)
    t = k*T;
    
    xdt = center(1) + R1*sin(2*gamma*t);
    ydt = center(2) + R2*sin(gamma*t);
    
    xd_dott = 2*R1*gamma*cos(2*gamma*t);
    yd_dott = (gamma)*(R2)*cos(gamma*t);fi([], 1, 16)
    thetadt = atan2(yd_dott, xd_dott);
    
    xd_dot_dott = -4*(gamma^2)*R1*sin(2*gamma*t);
    yd_dot_dott = (gamma^3)*(-R2)*cos(gamma*t);
    
    vkd= sqrt(xd_dott^2 + yd_dott^2);
    tmp = yd_dot_dott*xd_dott - xd_dot_dott*yd_dott;
    omegadt = tmp/(vdt^2);
    
 
        pause(T);   %Pause for T seconds
        
    Leader_velocity.Linear.X = vkd;    %Forward Velocity [m/sec]
    Leader_velocity.Angular.Z = omegadt;   %Angular Velocity [rad/sec]
    send(Leader_cmd_Pub,Leader_velocity);     %Send both Forward and Angular velocity to robot
    
    Leader_pose = receive(Leader_pose_Sub);
         leadX = Leader_pose.Pose.Pose.Position.X;
         leadY = Leader_pose.Pose.Pose.Position.Y;
         leadT = Leader_pose.Pose.Pose.Orientation.W;
         leadV = Leader_velocity.Linear.X;
         leadW = Leader_velocity.Angular.Z;
    if(k >= Tausteps)
    dtnew = t;
    
    

                kpr = k - Tausteps + 1;
        Follow_pose = receive(Follow_pose_Sub);
         flowX = Follow_pose.Pose.Pose.Position.X;
         flowY = Follow_pose.Pose.Pose.Position.Y;
         flowT = Follow_pose.Pose.Pose.Orientation.W;
        e_xpr = (leadX-flowX)*cos(flowT)+(leadY-flowY)*sin(flowT);
        e_ypr = -(leadX-flowX)*sin(flowT)+(leadY-flowY)*cos(flowT);
        e_thetapr = leadT - flowT;
        

        k1 = 1.7;
        k2 = ((a^2) - ((leadW)^2))/(abs(leadV));
        k3 = k1;
        
        u1pr = -k1*e_xpr;
        u2pr = -k2*e_ypr - k3*e_thetapr;
        
        vkdpr = Leader_velocity.Linear.X*cos(e_thetapr)-u1pr;
        omegadtpr = Leader_velocity.Linear.X - u2pr;
        Follow_velocity.Linear.X = vkdpr;    %Forward Velocity [m/sec]
   	    Follow_velocity.Angular.Z = omegadtpr;   %Angular Velocity [rad/sec]    
        send(Follow_cmd_Pub,Follow_velocity);     %Send both Forward and Angular velocity to robot
        %receive(Follow_setPose_Pub,Follow_pose);
       
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




















