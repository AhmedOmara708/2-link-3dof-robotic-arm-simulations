%% Offline Forward kinematics task
%{ 
Using the DH_template function, calculate the transform from the base frame
to the end-effector frame. Input test values for the DH parameters and
compare the results with the given table.
%}

% theta = Joint Angle
% d = Offset
% a = Length
% alpha = Angle about common normal

q = [-0.92; -0.25; 3.14];

init = [0;0;0;1];

H_1 = DH_template(q(1,1),0,-pi/2,0);
H_2 = DH_template(q(2,1),0,0,13.2);
H_3 = DH_template(q(3,1) - pi/2,0,0,13.2); %alpha = -pi/2

H = H_1*H_2*H_3;
Final = H*init;

disp(H)
disp(Final)