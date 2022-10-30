%% Inverse kinematics task
%{ 
Load a dataset of joint variables and process as in the forward kinematics
task to calculatre the end-effector position, use this position as the
input for the inverse kinematics calculation and compare the result.
%}

% theta = Joint Angle
% d = Offset
% a = Length
% alpha = Angle about common normal
% M = Input transformation matrix

theta = load('inverse_kinematics.csv');

%% Forward Kinematics
% Determine end effector position using forward kinematics as in Task 4
% Write forward kinematics code here:

init = [0;0;0;1];
H_prev = eye(4);
%H_stored(9836,16) = [];
%coord(9836,1) = []; 

for i = 1:9836
    
    %%Forward
    
    theta_1 = theta(i,1);
    theta_2 = theta(i,2);
    theta_3 = theta(i,3);
    [H_1,~,~] = DH_template(theta_1,0,-pi/2,0);
    [H_2,~,~] = DH_template(theta_2,0,0,13.2);
    [H_3,~,~] = DH_template(theta_3 - pi/2,0,0,13.2);
    [~,R_1,~] = DH_template(theta_1,0,-pi/2,0);
    [~,R_2,~] = DH_template(theta_2,0,0,13.2);
    [~,R_3,~] = DH_template(theta_3 - pi/2,0,0,13.2);    
    [~,~,T_1] = DH_template(theta_1,0,-pi/2,0);
    [~,~,T_2] = DH_template(theta_2,0,0,13.2);
    [~,~,T_3] = DH_template(theta_3 - pi/2,0,0,13.2);      
    
   %{A_1 = H_1; 
    
    %H_1(1) = A_1;
    %H_2(1) = A_2;
    %H_3(1) = A_3;
    %H_1(2) = R_1;
    %H_2(2) = R_2;
    %H_3(2) = R_3;
    %H_1(3) = T_1;
    %H_2(3) = T_2;
    %H_3(3) = T_3;    
    
    H = H_1*H_2*H_3;
    %H_stored = H_prev * H ;
    %H_prev = H_stored;
        
    H_stored(i,1) = H(1,1);
    H_stored(i,2) = H(1,2);
    H_stored(i,3) = H(1,3);
    H_stored(i,4) = H(1,4);
    H_stored(i,5) = H(2,1);
    H_stored(i,6) = H(2,2);
    H_stored(i,7) = H(2,3);
    H_stored(i,8) = H(2,4);
    H_stored(i,9) = H(3,1);
    H_stored(i,10) = H(3,2);
    H_stored(i,11) = H(3,3);
    H_stored(i,12) = H(3,4);
    H_stored(i,13) = H(4,1);
    H_stored(i,14) = H(4,2);
    H_stored(i,15) = H(4,3);
    H_stored(i,16) = H(4,4);
    
    Final = H*init;
    coord(i,1) = Final(1,1);
    coord(i,2) = Final(2,1);
    coord(i,3) = Final(3,1);
   
    %disp(H)
    %disp(Final)
    %figure(1);
    %plot3(Final(1,1), Final(2,1), Final(3,1), '.'); hold on, grid on, 
    %xlabel('x')
    %ylabel('y')
    %zlabel('z')
    
    %%Inverse
    
    x = coord(i,1);
    y = coord(i,2);
    z = coord(i,3);
    l_1 = 13.2;
    l_2 = 13.2;
    R = sqrt(x^2+y^2);
    a = sqrt(x^2+y^2+z^2);
    angle_triangle = acos(((l_1)^2 + (l_2)^2 - a^2)/(2 * l_1 * l_2));
    angle_cone_1 = acos(z/a);
    angle_cone_2 = acos(((l_1)^2 + (a)^2 - (l_2)^2)/(2 * l_1 * a));
    
    q_1 = atan(y/x);
    q_2 = -pi/2 + (angle_cone_1 - angle_cone_2);
    q_3 = 2*pi - angle_triangle - pi/2;
    
    q_stored(i,1) = q_1;
    q_stored(i,2) = q_2;
    q_stored(i,3) = q_3;
    
    %%Jacobian
    
    T = [H(1,4); H(2,4); H(3,4)];
    
    J_1 = cross(eye(3)*[0;0;1],(T - [0;0;0]));
    J_2 = cross(R_1*[0;0;1],(T - T_1));
    J_3 = cross(R_2*[0;0;1],(T - T_2));
    
    J = [J_1 J_2 J_3];
    J_stored(i,1) = J(1,1);
    J_stored(i,2) = J(1,2);
    J_stored(i,3) = J(1,3);
    
end
disp(J_stored)

%% Inverse Kinematics
% Calculate joint values based on the end effector position and compare the
% difference between the calculated and real values
% Write inverse kinematics code here:

%q_1(9836) = [];
%q_2(9836) = [];
%q_3(9836) = [];



%for j = 1:9836
%
%    x = coord(j,1);
%    y = coord(j,2);
%    z = coord(j,3);
%    l_1 = 13.2;
%    l_2 = 13.2;
%    R = sqrt(x^2+y^2);
%    a = sqrt(x^2+y^2+z^2);
%    angle_triangle = acos(((l_1)^2 + (l_2)^2 - a^2)/(2 * l_1 * l_2));
%    angle_cone_1 = acos(z/a);
%    angle_cone_2 = acos(((l_1)^2 + (a)^2 - (l_2)^2)/(2 * l_1 * a));
%    
%    q_1 = atan(y/x);
%    q_2 = -pi/2 + (angle_cone_1 - angle_cone_2);
%    q_3 = 2*pi - angle_triangle - pi/2;
%    
%end

disp(q_1)
disp(q_2)
disp(q_3)
%disp(H_stored)

%figure(1);
%plot3(coord(1,:), coord(2,:), coord(3,:)); hold on, grid on

diff1 = q_stored(:,1) - theta(:,1);
diff2 = q_stored(:,2) - theta(:,2);
diff3 = q_stored(:,3) - theta(:,3);

figure(2);
plot(diff1); hold on, grid on
ylabel('q1 - theta_1')
xlabel('n')
    
figure(3);
plot(diff2); hold on, grid on
ylabel('q2 - theta_2')
xlabel('n')
    
figure(4);
plot(diff3); hold on, grid on
ylabel('q3 - theta_3')
xlabel('n')
