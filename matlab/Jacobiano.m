clear all; clc; close all
%% DH modificado (Craig): [a_(i-1), alpha_(i-1), d_i, theta_i]
q = [0 0 0 0 0 0];              % <-- angulos de /joint_states

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

T_ROS = [1 0 0 0.465; 0 1 0 0.000; 0 0 1 0.695; 0 0 0 1];
disp('Transformacion obtenida de ROS2:'); disp(T_ROS)

fprintf('Error de posicion    = %.9f m\n', norm(T_base_flange(1:3,4)-T_ROS(1:3,4)));
R_err = T_ROS(1:3,1:3)' * T_base_flange(1:3,1:3);
fprintf('Error de orientacion = %.9f rad\n', ...
    acos(max(-1,min(1,(trace(R_err)-1)/2))));

%% ===== PARTE 5: Jacobiano geometrico desde el modelo DH =====

J = jacobiano_dh(T, T_corr, frame);
J(:,3:6) = -J(:,3:6);        % convencion: en la tabla DH theta_i = -q_i para i=3..6
disp('Jacobiano geometrico (MATLAB, desde DH):'); disp(J)

% Pegar aqui el Jacobiano que imprime KDL/MoveIt para la misma q
J_KDL = [ ...
    0      0.365 -0.035  0      0      0;
    0.465  0      0      0      0      0;
    0     -0.415  0.415  0      0.080  0;
    0      0      0     -1      0     -1;
    0      1     -1      0     -1      0;
    1      0      0      0      0      0 ];


fprintf('Error maximo J_DH vs J_KDL = %.3e\n', max(abs(J(:)-J_KDL(:))));

% Indice de manipulabilidad de Yoshikawa (detecta singularidades)
fprintf('Manipulabilidad w = sqrt(det(J*J'')) = %.6f\n', sqrt(det(J*J')));
fprintf('Numero de condicion = %.3e\n', cond(J));

function T_ij = T_DH(a_ij, alpha_ij, d_i, theta_i)
T_ij = [ cos(theta_i),                -sin(theta_i),                0,             a_ij;
    cos(alpha_ij)*sin(theta_i),   cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -d_i*sin(alpha_ij);
    sin(alpha_ij)*sin(theta_i),   sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  d_i*cos(alpha_ij);
    0, 0, 0, 1 ];
end
function J = jacobiano_dh(T, T_corr, frame)
% Craig (DH modificado): z_i y p_i salen de T_0i, no de T_0,(i-1)
T_e = T(:,:,frame) * T_corr;
p_e = T_e(1:3,4);
J = zeros(6, frame);
for i = 1:frame
    z = T(1:3,3,i);        % eje del joint i, expresado en base
    p = T(1:3,4,i);        % punto sobre el eje del joint i
    J(1:3,i) = cross(z, p_e - p);
    J(4:6,i) = z;
end
end