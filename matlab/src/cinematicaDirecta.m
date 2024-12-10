syms alpha_i_j beta_i_j gamma_i_j x_i_j y_i_j z_i_j

% Datos iniciales
z_0_1 = 0.362;
theta_0_1 = 0.5;
theta_1_2 = 0.5;
z_2_3 = 0.22;
theta_2_3 = 0.5;
z_3_4 = 0.22;
theta_3_4 = 0.5;
z_4_p = 0.1475;

% Transformaciones homogéneas intermedias
T_0_1 = T_i_j(0, 0, z_0_1, 0, 0, theta_0_1);
T_1_2 = T_i_j(0, 0, 0, 0, theta_1_2, 0);
T_2_3 = T_i_j(0, 0, z_2_3, 0, theta_2_3, 0);
T_3_4 = T_i_j(0, 0, z_3_4, 0, theta_3_4, 0);
T_4_p = T_i_j(0, 0, z_4_p, 0, 0, 0);

% Transformación homogénea final
T_0_P = T_0_1*T_1_2*T_2_3*T_3_4*T_4_p;

% Vector de postura 
xi_0_P = [T_0_P(1,4); T_0_P(2,4); T_0_P(3,4); atan2(T_0_P(3,2), T_0_P(3,3)); atan2(-T_0_P(3,1), sqrt(T_0_P(3,2)^2 + T_0_P(3,3)^2)); atan2(T_0_P(2,1), T_0_P(1,1))]

% Función para calcular la matriz de transformación homogénea
function T = T_i_j(x_i_j, y_i_j, z_i_j, gamma_i_j, beta_i_j, alpha_i_j)
    T = [cos(alpha_i_j)*cos(beta_i_j), cos(alpha_i_j)*sin(beta_i_j)*sin(gamma_i_j)-sin(alpha_i_j)*cos(gamma_i_j), sin(alpha_i_j)*sin(gamma_i_j)+cos(alpha_i_j)*sin(beta_i_j)*cos(gamma_i_j), x_i_j;
         sin(alpha_i_j)*cos(beta_i_j), cos(alpha_i_j)*cos(gamma_i_j)+sin(alpha_i_j)*sin(beta_i_j)*sin(gamma_i_j), sin(alpha_i_j)*sin(beta_i_j)*cos(gamma_i_j)-cos(alpha_i_j)*sin(gamma_i_j), y_i_j;
        -sin(beta_i_j),                cos(beta_i_j)*sin(gamma_i_j),                                          cos(beta_i_j)*cos(gamma_i_j), z_i_j;
         0,                            0,                                                                    0,                          1];
end
