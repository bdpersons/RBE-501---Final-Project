  function pathOfPoints = ImageProcessing(FileName)
    I = imread(FileName);
    BW = imbinarize(I); 
    [B, I]= bwboundaries(BW);
    B = B(1)
    L = imrotate(I, 180);
    visboundaries(L, 'Color','b');
    %assignin('base','B',B)
    MaxVal = max([B{:,:}]);
    max_X = MaxVal(2);
    max_Y = MaxVal(1);
    
    pathOfPoints = [B, max_X, max_Y];
end
