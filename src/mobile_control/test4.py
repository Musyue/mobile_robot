   def Caculate_Position_Command(self,PositionData,Left_Right,Driver_Id):
        #left target :(0x23, 0x7a, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00)
        #right target:(0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
        LEFT=self.MobileDriver_Command.BASIC_LEFT_TARGET_POSITION_COMMAND
        RIGHT=self.MobileDriver_Command.BASIC_RIGHT_TARGET_POSITION_COMMAND
        NEWLEFT=()
        NEWRIGHT=()
        if PositionData<0:
            if Left_Right=='left' or Left_Right=='LEFT':
                position=hex(PositionData & 0xFFFFFFFF)
                newposition=[position[i:i+2] for i in range(0,len(position), 2)]
                NEWLEFT=LEFT[:4]+(int(newposition[4],16),int(newposition[3],16),int(newposition[2],16),int(newposition[1],16))
                print newposition,NEWLEFT
                return NEWLEFT
            elif Left_Right =="right" or Left_Right=="RIGHT":
                position=hex(PositionData & 0xFFFFFFFF)
                newposition=[position[i:i+2] for i in range(0,len(position), 2)]
                NEWRIGHT=RIGHT[:4]+(int(newposition[4],16),int(newposition[3],16),int(newposition[2],16),int(newposition[1],16))
                print newposition,NEWRIGHT
                return NEWRIGHT
            else:
                print("Please input LEFT/RIGHT or left/right  -----")
        elif PositionData>0:
            position=hex(PositionData).replace('0x','')
            # print position
            if len(position)==1:
                newposition=position.zfill(len(position)+3)
                tempnewposition=[newposition[i:i+2] for i in range(0,len(newposition), 2)]
                # print tempnewposition
            elif len(position)==2:
                newposition=position.zfill(len(position)+2)
                tempnewposition=[newposition[i:i+2] for i in range(0,len(newposition), 2)]
                # print tempnewposition
            elif len(position)==3:
                newposition=position.zfill(len(position)+1)
                tempnewposition=[newposition[i:i+2] for i in range(0,len(newposition), 2)]
                # print tempnewposition
            else:
                tempnewposition=[position[i:i+2] for i in range(0,len(position), 2)]
                # print tempnewposition
            if Left_Right=='left' or Left_Right=='LEFT':
                NEWLEFT=LEFT[:4]+(int(tempnewposition[1],16),int(tempnewposition[0],16))+LEFT[6:]
                # print NEWLEFT
                return NEWLEFT
            elif Left_Right =="right" or Left_Right=="RIGHT":
                NEWRIGHT=RIGHT[:4]+(int(tempnewposition[1],16),int(tempnewposition[0],16))+RIGHT[6:]
                # print NEWRIGHT
                return NEWRIGHT
            else:
                print("Please input LEFT/RIGHT or left/right  -----")
        else:
            if Left_Right=='left' or Left_Right=='LEFT':
                return LEFT
            elif Left_Right =="right" or Left_Right=="RIGHT":
                return RIGHT
            else:
                print("Please input LEFT/RIGHT or left/right  -----")