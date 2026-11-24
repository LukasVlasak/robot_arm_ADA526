# soutez vaha
# 45 cm od base
# 15 cm od zeme 

# aktualni kinematika
# link1: 220
# link2: 300 - aby se ohnul o 100 stupnu
# link3: 280 - aby se ohnul o 100 stupnu
# link4: 200, jde zvestit, zmensit uz moc ne

# 86 max muze pero cumet 27 MIN
# aktualne 70


# pocitam cca 6cm na joint, COM ve stredu
JOINT_L = 0.06



# pocitam s tim ze mam kruhovy prurez 35mm vnejsi a 25mm vnitrni a PALG v kg
LINK1_W = 0.150
LINK2_W = 0.173
LINK3_W = 0.093

# pocitam s nasi docasnou kinematikou v m
# COM ve stredu
LINK1_L = 0.3
LINK2_L = 0.28
LINK3_L = 0.19 # vlastne zbytecne ale nechavam

# vzdalenost link COM od osy otaceni
LINK1_R = 150
LINK2_R = 362
LINK3_R = 450



# vzdalenost motor COM od osy otaceni
#MOTOR1_R = JOINT_L + LINK1_L + JOINT_L
MOTOR2_R = JOINT_L + LINK2_L + JOINT_L
MOTOR3_R = MOTOR2_R # pokud se motor 3 otáčí kolem vodorovné osy (například osy Y) a gravitace působí dolů ve směru Z, tak pro výpočet gravitačního torque potřebuješ pouze vodorovnou vzdálenost těžiště od osy motoru.
END_EFFECTOR_R = MOTOR2_R

# https://robotis.us/products/dynamixel-xm430-w350-t?srsltid=AU7gw4WdXu83MEWZ_S2w-UYNEthsW0pRe4yIigRo7nuGApWx5hD06XKj&dxl1=XM430-W350-T&dxl2=XM430-W210-R
MOTOR_W = 0.082


# 30g + jointy cca 20g na kazdy - joint u endeeffectoru, takze treba 45g => 75g
END_EFFECTOR_W = 0.03
JOINT_W = 0.02


#JOINT0_R = JOINT_L / 2 # primo u motoru
#JOINT1_R = MOTOR1_R - (JOINT_L / 2)
JOINT2_R = MOTOR2_R - (JOINT_L / 2)
JOINT3_R = MOTOR3_R - (JOINT_L / 2)

R_ZAVAZI = END_EFFECTOR_R * ZAVAZI_W

# R linku
R_LINK = LINK2_R * LINK2_W + LINK3_R * LINK3_W 
# R motoru
R_MOTOR = MOTOR2_R * MOTOR_W + MOTOR3_R * MOTOR_W
# R jointu
R_JOINT = JOINT2_R * JOINT_W + JOINT3_R * JOINT_W + END_EFFECTOR_R * END_EFFECTOR_W
# celkova hmostnost
M = END_EFFECTOR_W + 3 * JOINT_W + 2 * MOTOR_W  + LINK2_W + LINK3_W + ZAVAZI_W

# COM celeho
Xcom = (R_LINK + R_MOTOR + R_JOINT + R_ZAVAZI) / M

print("Xcom: " + str(Xcom))


# Torque, T = F * sin(omega) * r
# r .... Xcom
# F .... m * g => M * 9.81
# omega .... 90 stupnu => sin(omega) = 1 (gravitace pusobi kolmo)
T = M * 9.81 * Xcom

print("Torque needed: " + str(T) + "Nm\n")
print("Stall torque mensiho motor: 4.1Nm\n")
print("Stall torque vetsiho motor: 7.3Nm\n")

# Stall torque mensiho motor: 4.1Nm

# Stall torque vetsiho motor: 7.3Nm

# Co potrebujem pro vetsi motor - rameno v nejhorsi pozici !!!!
# zavazi 2kg: 18.504799199999997Nm
# zavazi 3kg: Torque needed: 25.9603992Nm
# zavazi 4kg: Torque needed: 33.4159992Nm
# zavazi 5kg: Torque needed: 40.8715992Nm

# Co potrebujem pro mensi motor
# zavazi 2kg: Torque needed: 9.315576Nm
# zavazi 3kg: Torque needed: 13.239576000000003Nm
# zavazi 4kg: Torque needed: 17.163576000000003Nm
# zavazi 5kg: Torque needed: 21.087576000000002Nm

# update s ramenem ne v nejhorsi pozici:
# Závaží 2 kg:  VELKÝ motor (rameno) 10.57 Nm (145 % stall)  |  MALÝ motor (loket)  3.29 Nm ( 80 % stall)
# Závaží 3 kg:  VELKÝ motor (rameno) 14.98 Nm (205 % stall)  |  MALÝ motor (loket)  4.76 Nm (116 % stall)
# Závaží 4 kg:  VELKÝ motor (rameno) 19.40 Nm (266 % stall)  |  MALÝ motor (loket)  6.23 Nm (152 % stall)
# Závaží 5 kg:  VELKÝ motor (rameno) 23.81 Nm (326 % stall)  |  MALÝ motor (loket)  7.70 Nm (188 % stall)