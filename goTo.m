function [] = goTo( x, y, z, pitch, phi, gripper )
%GOTO takes position and drive the ZUArm to go to it
%   Detailed explanation goes here

clc; 
close all

%% setup link lengths

L1 = 8; %the link length
L2 = 8.1; 
L3 = 17.2; 

Lmax = L1 + L2 + L3;

Lmin = L1 + cos(-(2/3)*pi) * L2 - L3;

%% Make sure that the position parameters within the Ropot workspace
% x must be in range [-Lmax, Lmax]

if(abs(x) > Lmax)
    fprintf('x is out of range');
    return
end

%% calculate y limits given x, read y value and make sure it's within limits

yLimit = sqrt(Lmax^2-x^2); %L^2 = x^2 + y^2 + z^2, z minimum value = 0 ==> y^2 = L^2-x^2
if (abs(y) > yLimit)
    fprintf('y is out of range');
    return
end

%% calculate z limits given x,

r = sqrt(x^2 + y^2);
zmaxLimit = sqrt(Lmax^2 - r^2);
if (r < Lmin)
    zminLimit = sqrt(Lmin^2 - r^2);
else
    zminLimit = -sqrt((L2+L3)^2 - (L1-r)^2);
end
    
% solve approximation errors
if ~isreal(zminLimit)
    zminLimit = 0;
end
if ~isreal(zmaxLimit)
    zmaxLimit = 0;
end
    
if ((z < zminLimit) || (z > zmaxLimit))
    fprintf('z is out of range');
    return
end

%% check that roll angle within the joint angle limits

if ~isempty(phi)
    if (phi > 180 || phi < 0)
        fprintf('roll angle is out of range');
        return
    end
end

%% check gripper angle

if ~isempty(gripper)
    if ( gripper < 0 || gripper > 180)
        fprintf('gripper angle out of range\n');
    end
end

%% read pitch angle

if ~isempty(pitch)
    if (pitch < -180 || pitch > 180) %accept all values for testing
        fprintf('pitch angle out of range\n');
        return
    end
end
pitch = pitch * (pi/180); %degres to radian
    
%check pitch limits -not reliable and need modification-
pitchmax1 = acos((r+L1+L2) / L3);
pitchmin1 = acos((r - L1 - L2) / L3);

pitchmax2 = asin((z+L2)/L3);
pitchmin2 = asin((z-L1-L2)/L3);

pitchmax = min(pitchmax1, pitchmax2);
pitchmin = max(pitchmin1, pitchmin2);

%% Inverse kinematics 
errorFlag = 0;

outstr= [];

if y < 0
    theta0 = atan2(-y,-x);
else
    theta0 = atan2(y,x);
end
    
theta2 = acos(((r-L3*cos(pitch))^2+(z-L3*sin(pitch))^2-L1^2-L2^2)/(2*L1*L2));
theta1 = asin(((z-L3*sin(pitch))*(L1+L2*cos(theta2))-L2*sin(theta2)*(r-L3*cos(pitch)))/((r-L3*cos(pitch))^2+(z-L3*sin(pitch))^2));
if y < 0
    theta1 = pi - theta1;
end
    
% check for errors due to pitch out of range
if ~isreal(theta2) || ~isreal(theta1) || ~isreal(theta0)
    errorFlag = 1;
    fprintf('pitch angle out of range, chose another angle');
end
    
theta3 = pitch - theta2 - theta1;
                    
% check constraints
    
if (~errorFlag) %&& ((pitchmin < pitch) && (pitch < pitchmax))
    %check for joints angles constraints
    if (pi < theta0 || 0 > theta0)
        errorFlag = 1;
    end

    if (pi < theta1 || 0 > theta1)
        errorFlag = 1;
    end

    if (0 < theta2 || (-(2/3)*pi) > theta2)
        errorFlag = 1;
    end

    if (0 < theta3 || (-pi) > theta3)
        errorFlag = 1;
    end

    if ~errorFlag 
        %forward kinematics for douple check
        pitchC = theta1+theta2+theta3;
        rC = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(pitchC);
        zC = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(pitchC);
        xC = rC*cos(theta0);
        yC = rC*sin(theta0);
    
        if theta1 > pi/2
            xC = -xC;
            yC = -yC;
        end
    
        %consider small error due to approximation
        if(abs(pitchC-pitch)>0.1 || abs(zC-z)>0.1 || abs(xC-x)>0.1 || abs(yC-y)>0.1 || ~isreal(zC) || ~isreal(xC) || ~isreal(yC))
            fprintf('problem \n')
            errorFlag = 1;
        end
    end
end
%radian to degres to be suitable for servo motor
theta0 = round(theta0 * (180/pi));
theta1 = round(theta1 * (180/pi));
theta2 = round(theta2 * (180/pi));
theta3 = round(theta3 * (180/pi));

if theta0 < 0
    theta0 = -theta0;
end

if theta1 < 0
    theta1 = -theta1;
end

if theta2 < 0
    theta2 = -theta2;
end

if theta3 < 0
    theta3 = -theta3;
end

%output string that should be sent to arduino
outstr = [outstr, '0 ', num2str(theta0), ':1 ',int2str(theta1) ,':2 ',int2str(theta2) ,':3 ' ,int2str(theta3) ,':'];


if errorFlag
    errorFlag
end

if ~isempty(phi)
    theta4 = round(phi);
    outstr = [outstr, '4 ', num2str(theta4), ':'];
end

if ~isempty(gripper)
    outstr = [outstr, '5 ', num2str(gripper), ':'];
end

%% Connecting to Arduino via serial communication
% output string sent to Arduino is at form "0 theta0:1 theta1:2 theta2:3
% theta3:4 theta4:5 gripper" not all terms must appear
outstr
delete(instrfind)
global S
if (~errorFlag || 1)
    S = serial('COM28','BaudRate',9600,'timeOut',0.05);
    fopen(S);
    pause(2);
    fprintf(S, outstr);
    pause(1);
    fclose(S);
end


end

