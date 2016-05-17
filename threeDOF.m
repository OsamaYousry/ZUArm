clc; 
close all
clear all

%% read the input desired position from command line and make sure it's within the Ropot workspace

L1 =20; %the lenk length
L1str=num2str(L1);

%% recive x from user if he want the arm to change position, x must be in range [-l, l]

str = strcat('enter X value in range [-', L1str,', ',L1str,'], press enter if you want the arm to stay at it''s position\n');
x = input(str);
if ~isempty(x)
    while(abs(x) > L1)
        x = input(str);
    end
end

%% if user inputs x, calculate y limits given x, read y value and make sure it's within limits

if (~isempty(x))
    yLimit = sqrt(L1^2-x^2); %L^2 = x^2 + y^2 + z^2, z minimum value = 0 ==> y^2 = L^2-x^2
    yLimitStr = num2str(yLimit);
    str = strcat('enter Y value in range [0, ',yLimitStr,']\n');
    y = input(str);
    while(isempty(y) || y > yLimit || y < 0)
        y = input(str);
    end
end

%% if user inputs x, calculate z given x,y as z have exactly one possible value given x,y

if (~isempty(x))
    z = sqrt(L1^2-x^2-y^2);
    zstr = num2str(z);
    str = strcat('Z value must be =',zstr,'\n');
    fprintf(str);
end

%% read roll angle from command line, make sure it's within the joint angle limits

phi = input('enter roll angle in degrees in range [0, 180], press enter if you don''t want the arm to roll\n');
if ~isempty(phi)
    while(phi > 180 || phi < 0)
        phi = input('enter roll angle in degrees in range [0, 180]\n');
    end
end

%% read gripper angle

%gripper move by angle not just open/close
gripper = input('enter the angle of the gripper, press enter if you want it to stay as it is\n');
if ~isempty(gripper)
    while( gripper < 0 || gripper > 180)
        gripper = input('enter the angle of the gripper, press enter if you want it to stay as it is\n');
    end
end

%% Inverse kinematics 

outstr= '';
if ~isempty(x)
    r = sqrt(x^2 + y^2);
    theta0 = round((atan2(y,x)/pi)*180);
    theta1 = round((atan2(z,r)/pi)*180);
    outstr = strcat(outstr, {'0 '},num2str(theta0), {':1 '},int2str(theta1),':');
end

if ~isempty(phi)
    theta2 = round(phi);
    outstr = strcat({'2 '}, num2str(phi), ':');
end

if ~isempty(gripper)
    outstr = strcat(outstr, {'3 '}, num2str(gripper), ':');
end

%% Connecting to Arduino via serial communication
% output string sent to Arduino is at form "0 theta0:1 theta1:2 phi:3
% gripper" not all terms must appear

delete(instrfind)
global S
S = serial('COM28','BaudRate',9600,'timeOut',0.1);
fopen(S);
fprintf(S,outstr);
fclose(S);
