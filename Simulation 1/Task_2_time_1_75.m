d_1 = 9;
theta_1 = pi/4*(1.75-1);
a_1 = 0;
alpha_1 = -pi/2;

d_2 = 0;
theta_2 = 0;
a_2 = 16/3;
alpha_2 = 0;

d_3 = 0;
theta_3 = -pi/4;
a_3 = 15/4;
alpha_3 = 0;

H_1  = [cos(theta_1) -sin(theta_1)*cos(alpha_1) sin(theta_1)*sin(alpha_1) a_1*cos(theta_1); sin(theta_1) cos(theta_1)*cos(alpha_1) -cos(theta_1)*sin(alpha_1) a_1*sin(theta_1); 0 sin(alpha_1) cos(alpha_1) d_1; 0 0 0 1];
H_2  = [cos(theta_2) -sin(theta_2)*cos(alpha_2) sin(theta_2)*sin(alpha_2) a_2*cos(theta_2); sin(theta_2) cos(theta_2)*cos(alpha_2) -cos(theta_2)*sin(alpha_2) a_2*sin(theta_2); 0 sin(alpha_2) cos(alpha_2) d_2; 0 0 0 1];
H_3  = [cos(theta_3) -sin(theta_3)*cos(alpha_3) sin(theta_3)*sin(alpha_3) a_3*cos(theta_3); sin(theta_3) cos(theta_3)*cos(alpha_3) -cos(theta_3)*sin(alpha_3) a_3*sin(theta_3); 0 sin(alpha_3) cos(alpha_3) d_3; 0 0 0 1];

Init = [0; 0; 0; 1];
Transform = H_1*H_2*H_3;
Final = Transform*Init;

disp(Transform)
disp(Final)
