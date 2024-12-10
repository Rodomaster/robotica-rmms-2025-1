% Longitudes de los eslabones
l = [0.362, 0.22, 0.22, 0.1475];

% Vector de entrada
goal_pose = [0.3841, 0.2099, 0.6844, 0, 1.5, 0.5];

goal_position = goal_pose(1:3);
psi = goal_pose(4); theta = goal_pose(5); phi = goal_pose(6);

z_1 = [0; 0; 1];
z_3 = [0; 0; 1];
R_0_p = rotation_matrix(psi, theta, phi);
z_p_0 = R_0_p(:, 3);
p_1_0 = [0; 0; l(1)];
p_p_4 = [0; 0; l(3)];

joint_1 = atan2(goal_position(2), goal_position(1));

R_joint_1 = rotation_matrix(0, 0, joint_1);

p_4_0 = goal_position' - l(4) * z_p_0;

epsilon = atan2(sqrt(p_4_0(1)^2 + p_4_0(2)^2), p_4_0(3) - l(1));
joint_3 = acos((p_4_0(1)^2 + p_4_0(2)^2 + (p_4_0(3) - l(1))^2 - l(2)^2 - l(3)^2) / (2 * l(2) * l(3)));
alpha = atan2(l(3) * sin(joint_3), l(2) + l(3) * cos(joint_3));
joint_2 = epsilon - alpha;

R_0_3 = rotation_matrix(0, joint_2 + joint_3, joint_1);
R_3_4 = R_0_3' * R_0_p;
joint_4 = acos(R_3_4(1, 1));

joint_positions = [joint_1; joint_2; joint_3; joint_4]

function R = rotation_matrix(psi, theta, phi)
    R = [cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);
         sin(phi) * cos(theta), cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi);
         -sin(theta), cos(theta) * sin(psi), cos(theta) * cos(psi)];
end