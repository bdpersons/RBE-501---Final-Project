clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);
try
    %% Setup
    % Define all server IDs
    PID_SERV_ID = 01;
    PID_CONFIG_SERV_ID = 02;
    STATUS_SERV_ID = 10;
    DEBUG   = true;          % enables/disables debug prints
    
    packet = zeros(15, 1, 'single');

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

cellPoints = ImageProcessing('batman.png');
pixlePoints = cell2mat(cellPoints(1));
max_X = cell2mat(cellPoints(2));
max_Y = cell2mat(cellPoints(3));
worldPoints = point2world(pixlePoints, max_X, max_Y);

jointAngles = [];
tf = 0.01;
radiansToEncoder = 4095/(2*pi);
waypoints = [];

sizeWorld = size(worldPoints);

for k = 1:(sizeWorld(2)-1)
        tic
        packet = zeros(15, 1, 'single');
        
        q0 = worldPoints(1, k); % this is for joint 0
        q1 = worldPoints(2, k); % this is for joint 1
        q2 = worldPoints(3, k); % this is for joint 2
        
        q0f = worldPoints(1, k+1); % this is for joint 0
        q1f = worldPoints(2, k+1); % this is for joint 1
        q2f = worldPoints(3, k+1); % this is for joint 2
        
        a = quinticTG(0, tf, q0, q0f, 0,0,0,0);
        b = quinticTG(0, tf, q1, q1f, 0,0,0,0);
        c = quinticTG(0, tf, q2, q2f, 0,0,0,0);
        
        % will need to set positions with these for viaPts
        enc1 = quinticPositions(a);
        enc2 = quinticPositions(b);
        enc3 = quinticPositions(c); 
        
        all3Joints = [enc1; enc2; enc3];
        
        % each column in waypoints is encoder positions for a point.
        % 3 rows: encoder 1-3
        if (k==1)
            waypoints = all3Joints;
            
        else
            waypoints = [waypoints all3Joints];
        end
end

sizeWaypoints = size(waypoints);
for l = 1:sizeWaypoints(2)
    jointAngles = [jointAngles, ikin(waypoints(1:3,l))];
end

for c = 1:sizeWaypoints(2)
    packet(1) = jointAngles(1,c)*radiansToEncoder;
    packet(4) = jointAngles(2,c)*radiansToEncoder;
    packet(7) = jointAngles(3,c)*radiansToEncoder;  

    % set motor to those positions
    pp.write(PID_SERV_ID, packet);

    pause(0.05);
end 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
