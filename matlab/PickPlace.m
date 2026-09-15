clear all; clc; close all
%% DH modificado (Craig): [a_(i-1), alpha_(i-1), d_i, theta_i]
q = [-0.558598, 0.603141, -0.301604, -0.000000, -0.666052, 0.558598];              % <-- angulos de /joint_states

DH = [ ...
    0,     deg2rad(0),    0.330, q(1);              % 1
    0.050, deg2rad(-90),  0,     q(2)-pi/2;         % 2
    0.330, deg2rad(0),    0,    -q(3);              % 3
    0.035, deg2rad(-90),  0.335,-q(4);              % 4
    0,     deg2rad(90),   0,    -q(5);              % 5
    0,     deg2rad(-90),  0.080,-q(6)               % 6
    ];

T_corr = [0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1];   % marco DH_6 -> flange URDF

frame = size(DH,1);
for i = 1:frame
    T_ij(:,:,i) = T_DH(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
end
T(:,:,1) = T_ij(:,:,1);
for i = 1:frame-1
    T(:,:,i+1) = T(:,:,i) * T_ij(:,:,i+1);
end

T_base_flange = T(:,:,frame) * T_corr;
disp('Transformacion base_link -> flange (DH):'); disp(T_base_flange)

T_ROS = [-1 0 0 0.40; 0 1 0 -0.25; 0 0 -1 0.28; 0 0 0 1];
disp('Transformacion obtenida de ROS2:'); disp(T_ROS)
    
fprintf('Error de posicion    = %.9f m\n', norm(T_base_flange(1:3,4)-T_ROS(1:3,4)));
R_err = T_ROS(1:3,1:3)' * T_base_flange(1:3,1:3);
fprintf('Error de orientacion = %.9f rad\n', ...
    acos(max(-1,min(1,(trace(R_err)-1)/2))));

function T_ij = T_DH(a_ij, alpha_ij, d_i, theta_i)
T_ij = [ cos(theta_i),                -sin(theta_i),                0,             a_ij;
    cos(alpha_ij)*sin(theta_i),   cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -d_i*sin(alpha_ij);
    sin(alpha_ij)*sin(theta_i),   sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  d_i*cos(alpha_ij);
    0, 0, 0, 1 ];
end