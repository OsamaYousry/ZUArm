clc; 
close all
clear all

%% setup link lengths

L1 = 20; %the link length
L1str = num2str(L1);
L2 = 20; 
L2str = num2str(L2);
L3 = 20; 
L3str = num2str(L3);

Lmax = L1 + L2 + L3;
Lmaxstr = num2str(Lmax);

Lmin = L1 + cos(-(2/3)*pi) * L2 - L3;
Liminstr = num2str(Lmin); 

%% read the input desired position from command line and make sure it's within the Ropot workspace
% recive x from user if he want the arm to change position, x must be in range [-Lmax, Lmax]

str = strcat('enter x value in range [-', Lmaxstr,', ',Lmaxstr,'], press enter if you want the arm to stay at it''s position\n');
x = input(str);
if ~isempty(x)
    while(abs(x) > Lmax)
        x = input(str);
    end
end

%% if user inputs x, calculate y limits given x, read y value and make sure it's within limits

if (~isempty(x))
    yLimit = sqrt(Lmax^2-x^2); %L^2 = x^2 + y^2 + z^2, z minimum value = 0 ==> y^2 = L^2-x^2
    yLimitstr = num2str(yLimit);
    str = strcat('enter y value in range [-', yLimitstr, ', ', yLimitstr, ']\n');
    y = input(str);
    while(isempty(y) || abs(y) > yLimit)
        y = input(str);
    end
end

%% if user inputs x, calculate z given x,y as z have exactly one possible value given x,y

if (~isempty(x))
    r = sqrt(x^2 + y^2);
    zmaxLimit = sqrt(Lmax^2 - r^2);
    zmaxLimitstr = num2str(zmaxLimit);
    if r < Lmin
        zminLimit = sqrt(Lmin^2 - r^2);
    else
        zminLimit = -sqrt((L2+L3)^2 - (L1-r)^2);
    end
    zminLimitstr = num2str(zminLimit);
    
    str = strcat('enter z value in range [-', zminLimitstr, ', ', zmaxLimitstr, ']\n');
    z = input(str);
    while(isempty(z) || (z < zminLimit) || (z > zmaxLimit))
        z = input(str);
    end
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

%% read pitch angle

if (~isempty(x))
    pitchmax1 = acos((r+L1+L2) / L3);
    pitchmin1 = acos((r - L1 - L2) / L3);
    
    pitchmax2 = asin((z+L2)/L3);
    pitchmin2 = asin((z-L1-L2)/L3); 
    
    pitchmax = min(pitchmax1, pitchmax2)*(180/pi);
    pitchmin = max(pitchmin1, pitchmin2)*(180/pi);
    
    pitchmaxstr = num2str(pitchmax);
    pitchminstr = num2str(pitchmin);
    
    str = strcat('enter the angle of pitch orientation in range [', pitchminstr, ', ', pitchmaxstr, ']\n');
    pitch = input(str);
    while(isempty(pitch) || gripper < 0 || gripper > 360) %accept all values for testing
        pitch = input(str);
    end
    pitch = pitch * (pi/180);
end


%% Inverse kinematics 
errorFlag = 0;

outstr= '';
if ~isempty(x)
    theta0 = round(atan2(y,x)*(180/pi));
    theta2 = round(acos(((r-L3*cos(pitch))^2+(z-L3*sin(pitch))^2-L1^2-L2^2)/(2*L1*L2))*(180/pi));
    theta1 = round(asin(((z-L3*sin(pitch))*(L1+L2*cos(theta2))-L2*sin(theta2)*(r-L3*cos(pitch)))/((r-L3*cos(pitch))^2+(z-L3*sin(pitch))^2))*(180/pi));
    theta3 = pitch - theta2 - theta1;
    outstr = strcat(outstr, {'0 '},num2str(theta0), {':1 '},int2str(theta0),':');
    % check angle constraints
    if (180 < theta0 || 0 > theta0)
        errorflag = 1;
        fprintf('theta0 not in limts = %i\n', theta0);
    end
    
    if (120 < theta1 || 0 > theta1)
        errorflag = 1;
        fprintf('theta1 not in limts = %i\n', theta1);
    end
    
    if (120 < theta2 || 0 > theta2)
        errorflag = 1;
        fprintf('theta2 not in limts = %i\n', theta2);
    end
    
    if (120 < theta3 || 0 > theta3)
        errorflag = 1;
        fprintf('theta3 not in limts = %i\n', theta3);
    end
    
    
end

if ~isempty(phi)
    theta4 = round(phi);
    outstr = strcat({'4 '}, num2str(theta4), ':');
end

if ~isempty(gripper)
    outstr = strcat(outstr, {'5 '}, num2str(gripper), ':');
end

%% Connecting to Arduino via serial communication
% output string sent to Arduino is at form "0 theta0:1 theta1:2 theta2:3
% theta3:4 theta4:5 gripper" not all terms must appear

delete(instrfind)
global S
S = serial('COM28','BaudRate',9600,'timeOut',0.1);
fopen(S);
fprintf(S,outstr);
fclose(S);
