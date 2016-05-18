%% checking limits
clc
clear
close all
%% setup links lenghts
L1 = 5;
L2 = 5;
L3 = 5;

Lmax = L1 + L2 + L3;

Lmin = L1 + cos(-(2/3)*pi) * L2 - L3;

for x = -Lmax : 0.5 : Lmax
    yLimit = sqrt(Lmax^2-x^2);
    
    for y = -yLimit : 0.5 : yLimit
        r = sqrt(x^2 + y^2);
        zmaxLimit = sqrt(Lmax^2 - r^2);
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
        
        for z = zminLimit : 0.5 : zmaxLimit
            
        pitchmax1 = acos((r+L1+L2) / L3);
        pitchmin1 = acos((r - L1 - L2) / L3);

        pitchmax2 = asin((z+L2)/L3);
        pitchmin2 = asin((z-L1-L2)/L3);
        
        pitchmax = min(pitchmax1, pitchmax2);
        pitchmin = max(pitchmin1, pitchmin2);
        
        for pitch = 0 : 0.1 : 2*pi
            
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
            pitchout = 0;
            if ~isreal(theta2) || ~isreal(theta1) || ~isreal(theta0)
                pitchout = 1;
            end
            
            theta3 = pitch - theta2 - theta1;
            
            flag = 0;
            if ~pitchout && ((pitchmin < pitch) && (pitch < pitchmax))
            % check angle constraints
            if ~(pi < theta0 || 0 > theta0)
                %fprintf('theta0 not in limts = %i\n', theta0);
                %pitch
                flag = flag +1;
            end
    
            if ~(pi < theta1 || 0 > theta1)
                %fprintf('theta1 not in limts = %i\n', theta1);
                %pitch
                flag = flag +1;
            end
    
            if ~(0 < theta2 || (-(2/3)*pi) > theta2)
                %fprintf('theta2 not in limts = %i\n', theta2);
                %pitch
                flag = flag +1;
            end
    
            if ~(0 < theta3 || (-(2/3)*pi) > theta3)
                %fprintf('theta3 not in limts = %i\n', theta3);
                %pitch
                flag = flag +1;
            end
            if flag == 4
                pitchC = theta1+theta2+theta3;
                rC = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(pitchC);
                zC = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(pitchC);
                xC = rC*cos(theta0);
                yC = rC*sin(theta0);
                if theta1 > pi/2
                    xC = -xC;
                    yC = -yC;
                end
                if(abs(pitchC-pitch)>0.1 || abs(zC-z)>0.1 || abs(xC-x)>0.1 || abs(yC-y)>0.1 || ~isreal(zC) || ~isreal(xC) || ~isreal(yC))
                    fprintf('problem \n')
                else
                    fprintf('good\n');
                end
                
            end
            end
            
        end
        end
    end
end
