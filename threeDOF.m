clc; 
close all
clear all
    
L1 =20;
L1str=num2str(L1);
str = strcat('enter X value in range [-', L1str,', ',L1str,']\n');
x = input(str);
while((isempty(x)) || (abs(x) > L1))
    x = input(str);
end

yLimit = sqrt(L1^2-x^2);
yLimitStr = num2str(yLimit);
str = strcat('enter Y value in range [0, ',yLimitStr,']\n');
y = input(str);
while(isempty(y) || y > yLimit || y < 0)
    y = input(str);
end

z = sqrt(L1^2-x^2-y^2);
zstr = num2str(z);
str = strcat('Z value must be  ',zstr,'\n');
fprintf(str);

phi = input('enter roll angle in degrees in range [0, 180]\n');
while(isempty(phi) || phi > 180 || phi < 0)
    phi = input('enter roll angle in degrees in range [0, 180]\n');
end

gripper = input('enter 0 if you want the gripper to grip after reaching distination, 1 if you want it to release\n');
while(isempty(gripper) || gripper < 0 || gripper > 1)
    gripper = input('enter 0 if you want the gripper to grip after reaching distination, 1 if you want it to release\n');
end

r = sqrt(x^2 + y^2);
theta0 = (atan2(y,x)/pi)*180;
theta1 = (atan2(z,r)/pi)*180;
theta2 = phi;

delete(instrfind)
global S
S = serial('COM28','BaudRate',9600,'timeOut',0.1);
fopen(S);
fprintf(S,strcat(num2str(theta0), ' ', num2str(theta1), ' ', num2str(theta2), ' ', num2str(gripper)));
fclose(S);