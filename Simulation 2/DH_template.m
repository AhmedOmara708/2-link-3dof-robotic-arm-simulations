function [A,R,T] = DH_template(theta,d,alpha,a)

    % theta = Joint Angle
    % d = Offset
    % a = Length
    % alpha = Angle about common normal
    % M = Input transformation matrix

    A = zeros(4,4);
    R = zeros(3,3);
    T = zeros(3,1);

    % Write DH code here:

    H = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); 0 sin(alpha) cos(alpha) d; 0 0 0 1];
    
    A = H;
    R = [H(1,1) H(1,2) H(1,3); H(2,1) H(2,2) H(2,3); H(3,1) H(3,2) H(3,3)];
    T = [H(1,4); H(2,4); H(3,4)];
    
    %disp(A)
    %disp(R)
    %disp(T)
    
end