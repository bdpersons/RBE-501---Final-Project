function T = dynamical_model()
    % Create Symbolic Variables
    syms m1 m2 m3
    syms L1 L2 L3
    syms th1 th2 th3 real
    syms r1 r2 r3

    % Define Slist and Theta list for Manipulator
    Slist = [[0;0;1;0;0;0], ...
             [0;-1;0;L1;0;0], ...
             [0;-1;0;L1;0;-L2]];

    thetalist = [th1; th2; th3];

    % Caclulate Each Transformation Matrix Individually
    I = eye(3);
    T ={};
    for i = 1:size(thetalist)
        % Calculate Skew Symmetric Matrix and Velocity
        w = VecToso3(Slist(1:3,i));
        v = Slist(4:6,i);

        % Define current theta value
        theta = thetalist(i);

        % Calculate Rotation Matrix
        R = I + sin(theta)* w + (1-cos(theta))*w^2;

        % Calculate ES
        ES = (I*theta+(1-cos(theta))*w+(theta-sin(theta))*w^2)*v;

        % Define Transformation Matrix of Given Joint
        T{i} = [R, ES; 0,0,0,1];
    end

    % Define M-Matrices for Each Joint Configuration
    M01 = [1,0,0,0;
           0,0,-1,0;
           0,1,0,L1;
           0,0,0,1];

    M02 = [1,0,0,L2;
           0,0,-1,0;
           0,1,0,L1;
           0,0,0,1];

    M03 = [1,0,0,L2+L3;
           0,0,-1,0;
           0,1,0,L1;
           0,0,0,1];

    % Calculate Final Transformation Matrices
    T01 = T{1}*M01;
    T02 = T{1}*T{2}*M02;
    T03 = simplify(T{1}*T{2}*T{3}*M03);

    % Calculate Transformation Matrices to Center of Masses
    T1_c1 = [1,0,0,0;
             0,1,0,-L1+r1;
             0,0,1,0;
             0,0,0,1];
    T2_c2 = [1,0,0,-L2+r2;
             0,1,0,0;
             0,0,1,0;
             0,0,0,1];
    T3_c3 = [1,0,0,-L3+r3;
             0,1,0,0;
             0,0,1,0;
             0,0,0,1];

    % Calulate Final Transformation Matrices to Center of Masses
    T0_c1 = simplify(T01*T1_c1);
    T0_c2 = simplify(T02*T2_c2);
    T0_c3 = simplify(T03*T3_c3);

    % Calculate Each Link Jacobian
    Jv_c1 = jacobian(T0_c1(1:3,4),[th1,th2,th3]);
    Jv_c2 = jacobian(T0_c2(1:3,4),[th1,th2,th3]);
    Jv_c3 = jacobian(T0_c3(1:3,4),[th1,th2,th3]);

    Jw_c1 = [T01(1:3,3), zeros([3,1]), zeros([3,1])];
    Jw_c2 = [T01(1:3,3), T02(1:3,3), zeros([3,1])];
    Jw_c3 = [T01(1:3,3), T02(1:3,3), T03(1:3,3)];

    % Calculate Dv
    Dv = m1*(Jv_c1.')*Jv_c1 + m2*(Jv_c2.')*Jv_c2 + m3*(Jv_c3.')*Jv_c3

    % Determine Rotation Matrices
    Rc1 = T0_c1(1:3,1:3);
    Rc2 = T0_c2(1:3,1:3);
    Rc3 = T0_c3(1:3,1:3);

    % Create a symbolic Intertia matrices
    I1 = sym('I1', [3,3]);
    I2 = sym('I2', [3,3]);
    I3 = sym('I3', [3,3]);

    % Calculate Dw
    Dw = (Jw_c1.')*Rc1*I1*(Rc1.')*Jw_c1 + (Jw_c2.')*Rc2*I2*(Rc2.')*Jw_c2 + (Jw_c3.')*Rc3*I3*(Rc3.')*Jw_c3

    % Simplify Dv and Dw
    Dv = simplify(Dv);
    Dw = simplify(Dw);

    % Calculate D
    D = simplify(Dv+Dw)

    % Calculate K

    K = simplify((1/2)*[th1,th2,th3]*D*[th1;th2;th3])

    % Create Variables
    syms th1_dot th2_dot th3_dot

    % Create Theta list
    thetalist = [th1; th2; th3];

    % Create a Velocity Vector
    vel = [th1_dot;th2_dot;th3_dot];

    % Define C list
    Chris = [];

    % Create for Loop to Calculate Christoffel Symbols
    for i = 1:3
        for j = 1:3
            for k = 1:3
                Chris{i,j,k} = (1/2)*(diff(D(k,j), thetalist(i))+ diff(D(k,i), thetalist(j)) - diff(D(i,j), thetalist(k)));
            end
        end
    end

    % Calculate C matrix entries
    C11 = Chris(1,1,1)*vel(1) + Chris(2,1,1)*vel(2) + Chris(3,1,1)*vel(3);
    C12 = Chris(1,1,2)*vel(1) + Chris(2,1,2)*vel(2) + Chris(3,1,2)*vel(3);
    C13 = Chris(1,1,3)*vel(1) + Chris(2,1,3)*vel(2) + Chris(3,1,3)*vel(3);

    C21 = Chris(1,2,1)*vel(1) + Chris(2,2,1)*vel(2) + Chris(3,2,1)*vel(3);
    C22 = Chris(1,2,2)*vel(1) + Chris(2,2,2)*vel(2) + Chris(3,2,2)*vel(3);
    C23 = Chris(1,2,3)*vel(1) + Chris(2,2,3)*vel(2) + Chris(3,2,3)*vel(3);

    C31 = Chris(1,3,1)*vel(1) + Chris(2,3,1)*vel(2) + Chris(3,3,1)*vel(3);
    C32 = Chris(1,3,2)*vel(1) + Chris(2,3,2)*vel(2) + Chris(3,3,2)*vel(3);
    C33 = Chris(1,3,3)*vel(1) + Chris(2,3,3)*vel(2) + Chris(3,3,3)*vel(3);

    % Calculate Coriolis/Centripetal Coupling Matrix
    C = [C11, C12, C13;
         C21, C22, C23;
         C31, C32, C33]

    % Create Variables
    syms g

    % Calculate P Vectors
    P1 = m1 * g * T0_c1(3,4);
    P2 = m2 * g * T0_c2(3,4);
    P3 = m3 * g * T0_c3(3,4);

    % Calculate Total Potential Energy
    P = P1+P2+P3;

    % Calculate Gravity Vectors
    g1 = diff(P, th1);
    g2 = diff(P, th2);
    g3 = diff(P, th3);

    % Calculate Gravity Vector
    G = [g1;g2;g3]

    % Create Variables
    syms th1_ddot th2_ddot th3_ddot

    % Create Acceleration Vector
    acc = [th1_ddot;th2_ddot;th3_ddot];

    % Caculate Torques with Calculated Matrices
    Tl = simplify(expand(D*acc + C*vel + G))
end
