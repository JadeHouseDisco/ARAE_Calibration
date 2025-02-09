%% Init
clc
clear
close all

%%
F_L = 0.25;
% Ps_e = [-0.25*cos(pi/4);
%         -0.25*cos(pi/4);
%         0];
% Ps_w = [-0.25*cos(pi/4) + 0.25;
%         -0.25*cos(pi/4);
%         0];

% Ps_e = [0;
%         -0.25;
%         0];
% Ps_w = [0.25;
%         -0.25
%         0];

Ps_e = [-0.25*cos(pi/4);
        0;
        -0.25*cos(pi/4)];
Ps_w = [-0.25*cos(pi/4);
        0;
        -0.25*cos(pi/4)-0.25];

P_w = [Ps_w(1);
       Ps_w(2);
       Ps_w(3)];
P_e = [Ps_e(1);
       Ps_e(2);
       Ps_e(3)];
P_s = [0;
       0;
       0];

U_cal = sqrt(Ps_e(1)^2+Ps_e(2)^2+Ps_e(3)^2);
h4 = pi/2 - acos((F_L^2 + U_cal^2 - (norm(P_w - P_s))^2)/(2*F_L*U_cal));  % elbow flexion
h2 = asin(-P_e(3)/U_cal); % shoulder flexion ----X axis
h1 = atan2(P_e(1),-P_e(2)); % Z axis shoulder abduction
vf = Ps_w(1)*cos(h1)+Ps_w(2)*sin(h1);
h3 = atan2(-(Ps_w(1)*sin(h1)*sin(h2)-Ps_w(2)*cos(h1)*sin(h2)+Ps_w(3)*cos(h2)),vf); % shoulder rotation

disp(h1)
disp(h2)
disp(h3)
disp(h4)