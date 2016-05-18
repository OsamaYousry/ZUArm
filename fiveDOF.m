clc; 
close all
clear all

%% setup link lengths

L1 = 5; %the link length
L1str = num2str(L1);
L2 = 5; 
L2str = num2str(L2);
L3 = 5; 
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
    
    % solve approximation errors
    if ~isreal(zminLimit)
        zminLimit = 0;
    end
    if ~isreal(zmaxLimit)
        zmaxLimit = 0;
    end
    
    zminLimitstr = num2str(zminLimit);
    
    str = strcat('enter z value in range [', zminLimitstr, ', ', zmaxLimitstr, ']\n');
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
    
    str = strcat('enter the angle of pitch orientation in range [0, 360]\n');
    pitch = input(str);
    while(isempty(pitch) || pitch < 0 || pitch > 360) %accept all values for testing
        pitch = input(str);
    end
    pitch = pitch * (pi/180); %degres to radian
    
    %check pitch limits -not reliable and need modification-
    pitchmax1 = acos((r+L1+L2) / L3);
    pitchmin1 = acos((r - L1 - L2) / L3);

    pitchmax2 = asin((z+L2)/L3);
    pitchmin2 = asin((z-L1-L2)/L3);
        
    pitchmax = min(pitchmax1, pitchmax2);
    pitchmin = max(pitchmin1, pitchmin2);
end


%% Inverse kinematics 
errorFlag = 0;

outstr= '';
if ~isempty(x)
    
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
    
    if ~errorFlag %&& ((pitchmin < pitch) && (pitch < pitchmax))
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
    
        if (0 < theta3 || (-(2/3)*pi) > theta3)
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
    
    %output string that should be sent to arduino
    outstr = strcat(outstr, {'0 '},num2str(theta0), {':1 '},int2str(theta1), {':2 '},int2str(theta2), {':3 '},int2str(theta3),':');
end

if ~isempty(phi)
    theta4 = round(phi);
    outstr = strcat(outstr, {'4 '}, num2str(theta4), ':');
end

if ~isempty(gripper)
    outstr = strcat(outstr, {'5 '}, num2str(gripper), ':');
end

%% Connecting to Arduino via serial communication
% output string sent to Arduino is at form "0 theta0:1 theta1:2 theta2:3
% theta3:4 theta4:5 gripper" not all terms must appear

delete(instrfind)
global S
if ~errorFlag
    S = serial('COM28','BaudRate',9600,'timeOut',0.1);
    fopen(S);
    fprintf(S,outstr);
    fclose(S);
end