# -*- coding: utf-8 -*
# This script for Linux environment
import time
import serial 
# Checked with TF03,

#Dans le terminal utiliser la commande <<sudo dmesg | grep usb>> pour valider sur quel port est connecter le lidar
#S'il y a un probleme de permision de lecture utiliser la commande <<sudo usermod -a -G dialout $USER>> remplacer $USER par votre utilisateur
ser = serial.Serial('/dev/ttyUSB0', 115200)
#ser = serial.Serial("/dev/ttyAMA0", 115200)
#ser = serial.Serial("/dev/tty0", 115200)
#ser = serial.Serial("COM12", 115200)

def write_data():

    ser.reset_input_buffer()

    ser.write(0x54)
    ser.write(0x06)
    ser.write(0x03)
    ser.write(0xE8)

    ser.write(0x03)
    ser.write(0x4E)
    ser.write(0x00)
    ser.write(0x00)

#success：is same as command
#fail：there isn’t any reaction over 1s
    #counter = ser.in_waiting # count the number of bytes of the serial port
    #if counter == 8:
     #   bytes_serial = ser.read(8)
      #  print("Reponse :" + str(bytes_serial)) 

    ser.write(0x5A)
    ser.write(0x04)
    ser.write(0x11)
    ser.write(0x6F)

    ser.write(0x00)
    ser.write(0x00)
    ser.write(0x00)
    ser.write(0x00)


    #success：5A 05 11 00 70
    #fail：5A 05 11 ER SU 
    bytes_serial = ser.read(8)
    print("Reponse :" + str(int(bytes_serial[0]))) 


def read_data():
    while True:
        count =0
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # this portion is for python3
                print("Printing python3 portion")            
                distance = bytes_serial[2] + bytes_serial[3]*256
                print("Distance:" + str(count) + " - "+ str(distance) + "\n")
                ser.reset_input_buffer()

            if bytes_serial[0] == "Y" and bytes_serial[1] == "Y":
                distL = int(bytes_serial[2].encode("hex"), 16)
                distH = int(bytes_serial[3].encode("hex"), 16)
                distance = distL + distH*256
                print("Printing python2 portion")
                print("Distance:" + str(count) + " - "+ str(distance) + "\n")
                ser.reset_input_buffer()


if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        write_data()
        #read_data()
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser != None:
            ser.close()
            print("program interrupted by the user")