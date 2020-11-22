  
function worldPoints = point2world(pathOfPoints, max_X, max_Y)
    T = [0, 0, -1, 0;
         0, 1, 0, 0;
         -1, 0, 0, max_Y;
         0, 0, 0, 1];
     worldPoints=[];
     scaleValueX = 12/(max_X);
     scaleValueY = 12/(max_Y);
     zVal = 8;
     pointDimension = size(pathOfPoints);
     for i = 1:pointDimension(1)
         point = pathOfPoints(i, :);
         p = [point(1); point(2); 0; 1];
         rotatePoint = T * p;
         x = zVal;
         y =(rotatePoint(2)*scaleValueX) - 8;
         z = rotatePoint(3)*scaleValueY;
         worldPoints = [worldPoints,[x*25.4;y*25.4;z*25.4]];
     end
    
     figure;
     plot3(worldPoints(1,:),worldPoints(2,:),worldPoints(3,:));
     grid on
     title("Planned Trajectory for Ghost")
     xlabel("x axis")
     ylabel("y axis")
     zlabel("z axis")
end
