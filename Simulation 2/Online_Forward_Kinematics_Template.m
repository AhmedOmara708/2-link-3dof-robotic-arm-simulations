%% Online Forward kinematics task
%{ 
Load a dataset of joint variables and process using the DH_template
function to calculate the end-effector position, plot end-effector
position in 3D. 
%}

% theta = Joint Angle
% d = Offset
% a = Length
% alpha = Angle about common normal

theta = load('forward_kinematics.csv');

init = [0;0;0;1];
H_prev = eye(4);

for i = 1:9460
    theta_1 = theta(i,1);
    theta_2 = theta(i,2);
    theta_3 = theta(i,3);
    H_1 = DH_template(theta_1,0,-pi/2,0);
    H_2 = DH_template(theta_2,0,0,13.2);
    H_3 = DH_template(theta_3 - pi/2,0,0,13.2); %alpha = -pi/2
    H = H_1*H_2*H_3;
    %H_stored = H_prev * H ;
    %H_prev = H_stored;
    
    Final = H*init;
   
    disp(H)
    disp(Final)
    
    plot3(Final(1,1), Final(2,1), Final(3,1), '.'); hold on, grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
end

%theta_1 = theta(:,1);
%theta_2 = theta(:,2);
%theta_3 = theta(:,3);

%{
for i = 1:9460
    theta_1 = theta(i,1);
    theta_2 = theta(i,2);
    theta_3 = theta(i,3);
    H_1 = DH_template(theta_1,0,-pi/2,0);
    H_2 = DH_template(theta_2,0,0,13.2);
    H_3 = DH_template(theta_3 - pi/2,0,0,13.2); %alpha = -pi/2
    H = H_1*H_2*H_3;
    init = [0;0;0;1];
    Final = H*init;
    disp(H)
    disp(Final)
    
    view(3)
    
    plot3(Final(1,1), Final(2,1), Final(3,1));
    hold all
    
end
%}

%init = [0;0;0;1];

%H_1 = DH_template(theta_1,0,-pi/2,0);
%H_2 = DH_template(theta_2,0,0,13.2);
%H_3 = DH_template(theta_3 - pi/2,0,0,13.2); %alpha = -pi/2

%H = H_1*H_2*H_3;
%Final = H*init;

%disp(H)
%disp(Final)