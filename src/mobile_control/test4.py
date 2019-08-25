# import re
# def Caculate_Velocity_Command(VelocityData,Left_Right):
#     """
#     LEFT:(0x23, 0xff, 0x60, 0x00, 0x58, 0x02, 0x00, 0x00)
#     RIGHT:(0x23, 0xff, 0x68, 0x00, 0xe8, 0x03, 0x00, 0x00)
#     """
#     LEFT=(0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
#     RIGHT=(0x23, 0xff, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00)
#     NEWLEFT=()
#     NEWRIGHT=()
#     if VelocityData<0:
#         if Left_Right=='left' or Left_Right=='LEFT':
#             velocity=hex(VelocityData & 0xFFFFFFFF)
#             newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
#             NEWLEFT=LEFT[:3]+(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))
#             print newvelocity,NEWLEFT
#             return NEWLEFT
#         elif Left_Right =="right" or Left_Right=="RIGHT":
#             velocity=hex(VelocityData & 0xFFFFFFFF)
#             newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
#             NEWRIGHT=RIGHT[:3]+(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))
#             print newvelocity,NEWRIGHT
#             return NEWRIGHT
#         else:
#             print("Please input LEFT/RIGHT or left/right  -----")
#     elif VelocityData>0:
#         velocity=hex(VelocityData).replace('0x','')
#         if len(velocity)%2!=0:
#             newvelocity=velocity.zfill(len(velocity)+1)
#             tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
#             # print tempnewvelocity
#         else:
#             tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
#             print tempnewvelocity
#         if Left_Right=='left' or Left_Right=='LEFT':
#             NEWLEFT=LEFT[:3]+(int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+LEFT[5:]
#             print NEWLEFT#,(int(tempnewvelocity[1],16),int(tempnewvelocity[0],16)),tempnewvelocity[0]
#             return NEWLEFT
#         elif Left_Right =="right" or Left_Right=="RIGHT":
#             NEWRIGHT=RIGHT[:3]+(int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+RIGHT[5:]
#             print NEWRIGHT
#             return NEWRIGHT
#         else:
#             print("Please input LEFT/RIGHT or left/right  -----")
#     else:
#         if Left_Right=='left' or Left_Right=='LEFT':
#             return LEFT
#         elif Left_Right =="right" or Left_Right=="RIGHT":
#             return RIGHT
#         else:
#             print("Please input LEFT/RIGHT or left/right  -----")
# def toHex(dec):
#     x = (dec % 16)
#     digits = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, a, b, c, d, e, f]
#     rest = dec / 16
#     if (rest == 0):
#         return digits[x]
#     return toHex(rest) + digits[x]

# numbers = [0, 11, 16, 32, 33, 41, 45, 678, 574893]
# print [toHex(x) for x in numbers]
# print [hex(x) for x in numbers]
# # Caculate_Velocity_Command(1500,'right')
# n = int(input("Input any number"))
# hex_values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 'a', 'b', 'c', 'd', 'e', 'f']
# reversed_number = ""
# while n > 0:
#     remainder = n % 16
#     n -= remainder
#     n //= 16
#     reversed_number += str(hex_values[remainder])

# print(reversed_number[::-1])
from bitstring import Bits
kk=[67, 100, 96, 0, 40, 173, 138, 254]
for i in kk:
    print hex(i)
print ~(kk[7]<<24|kk[6]<<16|kk[5]<<8|kk[4]-1)&0xFFFFFFFF
# def HEX_String_List_To_Oct_Four(Hexstrlist):
#     print "Hexstrlist",Hexstrlist
#     temp=[]
#     for i in Hexstrlist:
#         temp.append(bin(i))
#     # print temp
    
#     hex04=temp[4]
#     hex05=temp[5]
#     hex06=temp[6]
#     hex07=temp[7]
#     print bin(hex07<<24+hex06<<16+hex05<<8+hex04)
#     # newtemp=''.join([hex07,hex06,hex05,hex04]).replace('0b','')
#     # kkk=''.join([hex07,hex06,hex05,hex04]).replace('0b','')
#     # print newtemp,type(newtemp)
#     # ,Bits(bin=kkk).int
#     # print 0x8000000&kkk
#     # for i in kkk:
#     #     print i
# HEX_String_List_To_Oct_Four(kk)