clc; clear all; close all


%% Building the enviroment of the system
clc; clear all; close all
addpath 'D:\Education\My UWashington\Fall-2016\Advanced Robotics\Term Project\Codes\'
I = imread('buildings_walla_plan.jpg');
s=size(I);
for i=1:s(2)
    m(:,i) = I(:,i);
end
map= imbinarize(m);


img = robotics.BinaryOccupancyGrid (~map,10);
s=size(map)/10

figure
show(img);

%%  Inflate the map with the robot size
% Assume our robot is 0.8 meter in radius
inflatedMap = copy(img);
inflate(inflatedMap, 0.8);
figure;
show(inflatedMap);

%% Building workspace with respect to the robot dimention
i=1; j=1;
while j<=s(1,1)
    i=1;
    while i<=s(1,2)
        map(j,i)=getOccupancy(inflatedMap, [i j]);
        i=i+1;     
    end
    j=j+1;
end



%% Goal coordinate for the first robot
xGoal1=[60;18];

%% Minimum possible immediate payoff 
u=[1 0 -1 0 ; 0 1 0 -1];
x=[1;1]; 

while x(1,1)<=s(1,2)
    x(2,1)=1;
    while x(2,1)<=s(1,1)
       
        i=1;
        while i<=4
            pf(1,i)=payF(x,u(:,i),xGoal1,map);
            i=i+1;
        end
        ValueFm1(x(2,1),x(1,1))=min(pf);
        xm1=x;
        plu=[0;1];
        x=x+plu;
    end
    plu=[1;0];
    x=x+plu;
end

%% Calculating Value Function
x=[1;1]; 
cg=1; k1=1;
ma=max(ValueFm1);
mi=min(ValueFm1);
spanm1=max(ma)-min(mi)
while cg>0.001
    k1
    while x(1,1)<=s(1,2)
        x(2,1)=1;
        while x(2,1)<=s(1,1)
            i=1;
            while i<=4
                pf(1,i)=payFunc(x,u(:,i),xGoal1,map)+theSum(ValueFm1,x,u(:,i),s);
                i=i+1;
            end
            ValueF(x(2,1),x(1,1))=0.9*max(pf);
            [M,upi]=max(pf);
            policy(x(2,1),x(1,1))=upi;
            
            plu=[0;1];
            x=x+plu;
        end
        plu=[1;0];
        x=x+plu;
    end
    ma=max(ValueF);
    mi=min(ValueF);
    span=max(ma)-min(mi);
    cg=abs(span-spanm1)    
    spanm1=span;
    ValueFm1=ValueF;
    k1=k1+1;
    x=[1;1];
end


            


%% THE SECOND ROBOT


%% Goal coordinate
xGoal2=[40;40];

%% Minimum possible immediate payoff 
u=[1 0 -1 0 ; 0 1 0 -1];
x=[1;1]; 

while x(1,1)<=s(1,2)
    x(2,1)=1;
    while x(2,1)<=s(1,1)
       
        i=1;
        while i<=4
            pf(1,i)=payF(x,u(:,i),xGoal2,map);
            i=i+1;
        end
        ValueFm12(x(2,1),x(1,1))=min(pf);
        xm1=x;
        plu=[0;1];
        x=x+plu;
    end
    plu=[1;0];
    x=x+plu;
end

%% Calculating Value Function
x=[1;1]; 
cg=1; k2=1;
ma=max(ValueFm12);
mi=min(ValueFm12);
spanm12=max(ma)-min(mi)
while cg>0.001
    k2
    while x(1,1)<=s(1,2)
        x(2,1)=1;
        while x(2,1)<=s(1,1)
            i=1;
            while i<=4
                pf(1,i)=payFunc(x,u(:,i),xGoal2,map)+theSum(ValueFm12,x,u(:,i),s);
                i=i+1;
            end
            ValueF2(x(2,1),x(1,1))=0.9*max(pf);
            [M,upi]=max(pf);
            policy2(x(2,1),x(1,1))=upi;
            
            plu=[0;1];
            x=x+plu;
        end
        plu=[1;0];
        x=x+plu;
    end
    ma=max(ValueF2);
    mi=min(ValueF2);
    span2=max(ma)-min(mi);
    cg=abs(span2-spanm12)    
    spanm12=span2;
    ValueFm12=ValueF2;
    k2=k2+1;
    x=[1;1];
end

%% Value function and policy for the exit point

%% Exit Goal coordinate
xGoalEx=[88;12];

%% Minimum possible immediate payoff 
u=[1 0 -1 0 ; 0 1 0 -1];
x=[1;1]; 

while x(1,1)<=s(1,2)
    x(2,1)=1;
    while x(2,1)<=s(1,1)
       
        i=1;
        while i<=4
            pf(1,i)=payF(x,u(:,i),xGoalEx,map);
            i=i+1;
        end
        ValueFm1Ex(x(2,1),x(1,1))=min(pf);
        xm1=x;
        plu=[0;1];
        x=x+plu;
    end
    plu=[1;0];
    x=x+plu;
end

%% Calculating Value Function
x=[1;1]; 
cg=1; k3=1;
ma=max(ValueFm1Ex);
mi=min(ValueFm1Ex);
spanm1Ex=max(ma)-min(mi)
while cg>0.001
    k3
    while x(1,1)<=s(1,2)
        x(2,1)=1;
        while x(2,1)<=s(1,1)
            i=1;
            while i<=4
                pf(1,i)=payFunc(x,u(:,i),xGoalEx,map)+theSum(ValueFm1Ex,x,u(:,i),s);
                i=i+1;
            end
            ValueFEx(x(2,1),x(1,1))=0.9*max(pf);
            [M,upi]=max(pf);
            policyEx(x(2,1),x(1,1))=upi;
            
            plu=[0;1];
            x=x+plu;
        end
        plu=[1;0];
        x=x+plu;
    end
    ma=max(ValueFEx);
    mi=min(ValueFEx);
    spanEx=max(ma)-min(mi);
    cg=abs(spanEx-spanm1Ex)    
    spanm1Ex=spanEx;
    ValueFm1Ex=ValueFEx;
    k3=k3+1;
    x=[1;1];
end

%% -------------------------------------------------


%% Simulation of both

%% Drive a path from policy

figure
show(img);

x=[5;38];
x2=[1;34];

path=x;
j=2;
 while (norm(x-xGoal1)~=0) 
       upi=policy(x(2,1),x(1,1));
       path(:,j)=u(:,upi)+x
       x=path(:,j);
       j=j+1;
       if j==50
          %break
       end
           
 end
 
 path=path';

 
 %% Creat PurePursuit controller for robot1  to follow path
controller = robotics.PurePursuit;
controller.Waypoints = path;

% Define the start point and the goal based on the current location
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
startPoint = [robotCurrentLocation, 0];

controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than
% Linear velocity for a smooth path. 
controller.LookaheadDistance = 0.6;

%% Drivethe path from policy for the second robot
 path2=x2;
j=2;
 while (norm(x2-xGoal2)~=0) 
       upi=policy2(x2(2,1),x2(1,1));
       path2(:,j)=u(:,upi)+x2
       x2=path2(:,j);
       j=j+1;           
 end
 
 path2=path2';

%% Creat PurePursuit controller for robot2  to follow path
controller2 = robotics.PurePursuit;
controller2.Waypoints = path2;

% Define the start point and the goal based on the current location
robotCurrentLocation2 = path2(1,:);
robotGoal2 = path2(end,:);
startPoint2 = [robotCurrentLocation2, 0];

controller2.DesiredLinearVelocity = 0.3;
controller2.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than
% Linear velocity for a smooth path. 
controller2.LookaheadDistance = 0.6;

%% Creating a simulated robot for control

robot = ExampleHelperDifferentialDriveRobot(startPoint);

robot2 = ExampleHelperDifferentialDriveRobot(startPoint2);

%% Show the 2 robots running on the path
goalRadius = 0.1; %the robot stops when it is within this far from the goal
distanceToGoal = norm(robotCurrentLocation - robotGoal);
distanceToGoal2 = norm(robotCurrentLocation2 - robotGoal2);

j=1;
while (distanceToGoal > goalRadius)
    %compute the controller outputs, i.e. the input
    [v, omega]= step(controller, robot.CurrentPose);
    % Simulate the robot using the controller output
    drive(robot, v,omega)
    %Extract current location information ([X,Y])
    %robot
    robotCurrentLocation = robot.CurrentPose(1:2);
    %Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
    
    if norm(robotCurrentLocation-robotCurrentLocation2)<5
        continue
    else
    
        if distanceToGoal2< goalRadius
            continue
        else
            
            [v2, omega2]= step(controller2, robot2.CurrentPose);
            drive(robot2, v2,omega2)
            robotCurrentLocation2 = robot2.CurrentPose(1:2);
            distanceToGoal2 = norm(robotCurrentLocation2 - robotGoal2);
        end
        
    end
end


    


%% Simulation of both to Exit
pause(5)
%% Drive a path from policy
x=xGoal1;
x2=xGoal2;


path=x;
j=2;
 while (norm(x-xGoalEx)~=0) 
       upi=policyEx(x(2,1),x(1,1));
       path(:,j)=u(:,upi)+x
       x=path(:,j);
       j=j+1;
       if j==50
          %break
       end
           
 end
 
 path=path';

 
 %% Creat PurePursuit controller for robot1  to follow path
controller = robotics.PurePursuit;
controller.Waypoints = path;

% Define the start point and the goal based on the current location
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
startPoint = [robotCurrentLocation, 0];

controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than
% Linear velocity for a smooth path. 
controller.LookaheadDistance = 0.6;

%% Drivethe path from policy for the second robot
 path2=x2;
j=2;
 while (norm(x2-xGoalEx)~=0) 
       upi=policyEx(x2(2,1),x2(1,1));
       path2(:,j)=u(:,upi)+x2
       x2=path2(:,j);
       j=j+1;    
       if j==200
           break
       end
 end
 
 path2=path2';

%% Creat PurePursuit controller for robot2  to follow path
controller2 = robotics.PurePursuit;
controller2.Waypoints = path2;

% Define the start point and the goal based on the current location
robotCurrentLocation2 = path2(1,:);
robotGoal2 = path2(end,:);
startPoint2 = [robotCurrentLocation2, 0];

controller2.DesiredLinearVelocity = 0.3;
controller2.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than
% Linear velocity for a smooth path. 
controller2.LookaheadDistance = 0.6;

%% Creating a simulated robot for control

robot = ExampleHelperDifferentialDriveRobot(startPoint);

robot2 = ExampleHelperDifferentialDriveRobot(startPoint2);

%% Show the 2 robots running on the path
goalRadius = 0.1; %the robot stops when it is within this far from the goal
distanceToGoal = norm(robotCurrentLocation - robotGoal);
distanceToGoal2 = norm(robotCurrentLocation2 - robotGoal2);

j=1;
while (distanceToGoal2 > goalRadius)
    [v2, omega2]= step(controller2, robot2.CurrentPose);
    drive(robot2, v2,omega2)
    robotCurrentLocation2 = robot2.CurrentPose(1:2);
    distanceToGoal2 = norm(robotCurrentLocation2 - robotGoal2);
    
    
    if norm(robotCurrentLocation-robotCurrentLocation2)<4
        continue
    else
    
        if distanceToGoal< goalRadius
            continue
        else
            [v, omega]= step(controller, robot.CurrentPose);
            drive(robot, v,omega)
            robotCurrentLocation = robot.CurrentPose(1:2);
            distanceToGoal = norm(robotCurrentLocation - robotGoal);
        end
        
    end
end



%% Value Function plots for all three policys

figure()
x=1:s(1,2);
y=1:s(1,1);
mesh(x,y,ValueF)
xlabel('X [meters]')
ylabel('Y [meters]')
zlabel('Value Function')
figure()
mesh(x,y,ValueF2)
xlabel('X [meters]')
ylabel('Y [meters]')
zlabel('Value Function')
figure()
mesh(x,y,ValueFEx)
xlabel('X [meters]')
ylabel('Y [meters]')
zlabel('Value Function')

pause

%% Drive a path from policy

x=[10;10];
%x=[8;38];
%x=[80;20];
%x=[80;15];
x=[70;13];
x=[10;30];
x=[20;45];
x=[80;45];
%x=[85;35];
%x=[75;35];

path=x;
j=1;
 while (norm(x-xGoalEx)~=0) 
       upi=policyEx(x(2,1),x(1,1));
       path(:,j)=u(:,upi)+x
       x=path(:,j);
       j=j+1;
       if j==50
          %break
       end
           
 end
 
 path=path';

 
 %% Creat PurePursuit controller for robot1  to follow path
controller = robotics.PurePursuit;
controller.Waypoints = path;

% Define the start point and the goal based on the current location
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
startPoint = [robotCurrentLocation, 0];

controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than
% Linear velocity for a smooth path. 
controller.LookaheadDistance = 0.6;


%% Creating a simulated robot for control

robot = ExampleHelperDifferentialDriveRobot(startPoint);

%robot2 = ExampleHelperDifferentialDriveRobot(startPoint2);
%% Show the robot running on the path
goalRadius = 0.1; %the robot stops when it is within this far from the goal
distanceToGoal = norm(robotCurrentLocation - robotGoal);

j=1;
while (distanceToGoal > goalRadius)
    %compute the controller outputs, i.e. the input
    [v, omega]= step(controller, robot.CurrentPose);
    % Simulate the robot using the controller output
    drive(robot, v,omega)
    %Extract current location information ([X,Y])
    %robot
    robotCurrentLocation = robot.CurrentPose(1:2);
    %Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
        
end