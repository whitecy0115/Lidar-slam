function dirve12(v,omega,scout)
speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");

angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    

TxMsg = canMessage(273, false, 8);

TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);

transmit(scout,TxMsg);

end