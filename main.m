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

cellPoints = ImageProcessing('batman5.png');
pixlePoints = cell2mat(cellPoints(1));
max_X = cell2mat(cellPoints(2));
max_Y = cell2mat(cellPoints(3));
worldPoints = point2world(pixlePoints, max_X, max_Y);

jointAngles = [];

radiansToEncoder = 4095/(2*pi);

sizePoints = size(worldPoints);

for l = 1:sizePoints(2)
    jointAngles = [jointAngles, ikin(worldPoints(1:3,l))];
end

for k = 1:sizePoints(2)
        tic
        packet = zeros(15, 1, 'single');
        
        % will need to set positions with these for viaPts
        packet(1) = jointAngles(1, k)*radiansToEncoder; % this is for joint 0
        packet(4) = jointAngles(2, k)*radiansToEncoder; % this is for joint 1
        packet(7) = jointAngles(3, k)*radiansToEncoder; % this is for joint 2
        
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
