function Ts= FK_World(M,SArray,thetaArray)
    %Calculate the Foweard Kinematics of the robot using the product of
    %exponentials method. This function is taken from the last question of 
    %the previous homework.
    ES= 1;
    I= [1,0,0;0,1,0;0,0,1];
    for i=1:length(thetaArray)
        S=SArray(:,i);
        theta=thetaArray(i);
        Sw= [S(1), S(2), S(3)];
        Sv = [S(4); S(5); S(6)];
        w=[    0, -Sw(3), Sw(2);
             Sw(3), 0,  -Sw(1);
            -Sw(2), Sw(1), 0;];

        R= I + (sin(theta)*w) + ((1-cos(theta))*(w*w));
        V= (I*theta+(1-cos(theta))*w+((theta-sin(theta))*(w*w)))*Sv;
        Ei=[[R;0,0,0],[V;1]];
        ES=ES*Ei;
    end
    Ts=ES*M; 
end
