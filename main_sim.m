clear
clear java
clear classes;

M = [1, 0, 0,   175;
     0, 1, 0,  0;
     0, 0, 1, -34.28;
     0, 0, 0,  1];

%Set Screws for joints
S1=[0;0;1;0;0;0];
S2=[0;1;0;-135;0;0];
S3=[0;1;0;0;0;-175];

sArray = [S1, S2, S3];
thetaArray = [0, 0];

fkTransformation = FK_World(M, sArray, thetaArray);

cellPoints = ImageProcessing('batman5.png');
pixlePoints = cell2mat(cellPoints(1));
max_X = cell2mat(cellPoints(2));
max_Y = cell2mat(cellPoints(3));
worldPoints = point2world(pixlePoints, max_X, max_Y);

jointAngles = [];

robot = importrobot('./robot.urdf');

radiansToEncoder = 4095/(2*pi);

sizePoints = size(worldPoints);

for l = 1:sizePoints(2)
    jointAngles = [jointAngles, ikin(worldPoints(1:3,l))];
end

for k = 1:sizePoints(2)
        zero = homeConfiguration(robot);
        zero(1).JointPosition = jointAngles(1, k);
        zero(2).JointPosition = jointAngles(2, k);
        zero(3).JointPosition = jointAngles(3, k);
        show(robot, zero);
        hold on
        axis([-230 370 -200 200 -10 600]);
        hold off
        
        packet(1) = jointAngles(1, k)
        packet(4) = jointAngles(2, k)
        packet(7) = jointAngles(3, k)
        
        % set motor to those positions
        pp.write(PID_SERV_ID, packet);
        
       pause(0.001);
end
