def Caculate_Velocity_Command(VelocityData,flag):

    BASIC=(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)

    NEWCMD=()
    if flag:#steer
        VelocityData=VelocityData*512*4096/1875
    else:
        VelocityData=VelocityData*512*10000/1875
    if VelocityData<0:
        velocity=hex(VelocityData & 0xFFFFFFFF)
        newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
        print newvelocity
        NEWCMD=(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))+BASIC[4:]
        print newvelocity,NEWCMD
        return NEWCMD

    elif VelocityData>0:
        velocity=hex(VelocityData).replace('0x','')
        print velocity
        newvelocity=velocity.zfill(len(velocity)+8-len(velocity))
        print newvelocity
        tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
        print tempnewvelocity
        NEWCMD=(int(tempnewvelocity[3],16),int(tempnewvelocity[2],16),int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+BASIC[5:]
        print NEWCMD
        return NEWCMD

    else:
        pass
Caculate_Velocity_Command(-1000,1)