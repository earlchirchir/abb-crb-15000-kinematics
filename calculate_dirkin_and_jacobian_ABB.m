clear all
close all
clc

% Define symbolic variables for joint angles (q1-q6)
syms q1 q2 q3 q4 q5 q6 real % Define joint variables

% Define symbolic variables for the robot's physical link lengths (DH params)
syms d1 a2 a3 d4 a5 d6 real % Define DH parameters

% Create the DH parameter table where each row is [a, alpha, d, theta]
DH_table = [
    % Define DH table with symbolic variables
    0 -pi/2 d1 q1;
    a2 0 0 q2;
    a3 -pi/2 0 q3;
    0 pi/2 d4 q4;
    a5 -pi/2 0 q5;
    0 0 d6 q6;
];

fprintf('\n--- Printing DH Table ---\n');
disp(DH_table);


% Calculate the individual homogeneous transformation matrices for each link
A01 = DH(DH_table(1, :));
A12 = DH(DH_table(2, :));
A23 = DH(DH_table(3, :));
A34 = DH(DH_table(4, :));
A45 = DH(DH_table(5, :));
A56 = DH(DH_table(6, :));

fprintf('\n--- Printing Individual Transformation Matrices (A01 - A56) ---\n');
disp(A01);
disp(A12);
disp(A23);
disp(A34);
disp(A45);
disp(A56);

% Compute and simplify the end-effector transformation matrix
A06 = simplify(A01 * A12 * A23 * A34 * A45 * A56);

% Display the full symbolic transformation matrix from base to end-effector
fprintf('\n--- Printing Full Transformation Matrix (A06) ---\n');
disp('A06');
disp(A06);

% Extract the position coordinates from the 4th column of the transformation matrix
p_ee_ABB_symb = A06(1:3, 4);

% Plug in the specific physical robot link lengths (constants)
fprintf('\n--- Printing Substituted End-Effector Position (p_ee) ---\n');
% Substitute constants into the end-effector position
p_ee_ABB = subs(p_ee_ABB_symb, ...
    {d1, a2, a3, d4, a5, d6}, ...
    {0.265, 0.444, 0.110, 0.470, 0.08, 0.101});
disp(p_ee_ABB);

% Derive the Jacobian using symbolic partial derivatives of position wrt joint angles
J_ABB_symb = simplify(jacobian(p_ee_ABB_symb, [q1; q2; q3; q4; q5; q6]));

% Apply the same link length constants to the symbolic Jacobian
J_ABB = subs(J_ABB_symb, ...
    {d1, a2, a3, d4, a5, d6}, ...
    {0.265, 0.444, 0.110, 0.470, 0.08, 0.101});

% Convert the symbolic results to optimized .m function files for high-speed simulation
matlabFunction(J_ABB, 'File', 'jacobian_ABB');
matlabFunction(p_ee_ABB, 'File', 'dirkin_ABB');
