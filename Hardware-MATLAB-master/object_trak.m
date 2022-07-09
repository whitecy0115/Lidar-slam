function object_trak(cam,scout,detector)
load("Adress_Detector.mat");
scout = canChannel('PEAK-System', 'PCAN_USBBUS1');
start(scout)

message = receive(scout, Inf, "OutputFormat", "timetable");

TxMsg = canMessage(1057, false, 1);
TxMsg.Data = [1];
transmit(scout,TxMsg);

cam = webcam(3);

cam.Resolution = '640x480';
cx = round(640/2);
left_treshold = cx-50;
right_teshold = cx+50;

count = 0;
sampleTime = 0.54; 
vizRate = rateControl(1/sampleTime);
i=1;
figure;
while count <= 40
    
    if mod(i,3) == 0
    I = snapshot(cam);
    [bboxes,~] = detect(detector,I);
    else
        bboxes = [];
    end

    if isempty(bboxes) ==0
    centroids = round(bboxes(:, 1) + bboxes(:, 3) / 2);
    label{1} = num2str(scores);
    label{2} = 'center';
    bbox = [bboxes;320 240 5 5];
    img = insertObjectAnnotation(I,'rectangle',bbox,label);
    imshow(img);drawnow;
   
        if centroids >= right_teshold % 우회전
            v=0.2;
            omega = -0.2;
    
            speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
            angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
            TxMsg = canMessage(273, false, 8);
            TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
            transmit(scout,TxMsg);

        elseif centroids <= left_treshold % 좌회전
            v=0.2;
            omega = 0.2;
    
            speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
            angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
            TxMsg = canMessage(273, false, 8);
            TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
            transmit(scout,TxMsg);
        else
        v=0.2;
        omega = 0;
    
        speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
        angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
        TxMsg = canMessage(273, false, 8);
        TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
        transmit(scout,TxMsg);
        end
    else
        count = count+1;
    
        v=0.2;
        omega = 0;
    
        speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
        angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
        TxMsg = canMessage(273, false, 8);
        TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
        transmit(scout,TxMsg);
    end
    i=i+1;
    waitfor(vizRate);
   
end
end