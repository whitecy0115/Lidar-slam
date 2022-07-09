function turn_left(scout)
TxMsg = canMessage(273, false, 8);
TxMsg.Data = ([0 0 0 0 0 0 0 0]);
transmit(scout,TxMsg);
            
for i=1:5
    if i<=5
        v=0;
        omega= 0.3;
        speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
        angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");

        TxMsg = canMessage(273, false, 8);
        TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
        transmit(scout,TxMsg);

    pause(0.5);
    end
end
end