#!/usr/bin/env python3

"""
@Author: Miro
@Date: 18/10/2022
@Version: 1.0
@Objective: start the experiment 
@TODO:
"""

import serial
import time
import paramiko
import sys
import os

baud_target = 38400
baud_release = 9600

target_command = b"a"
dict_command_release = {'0': "SETUP", '1': "s1", '2': "s2", '3': "s3", '4': "s4"}

command_release_default = "SETUP"
port_release_default = "COM3"
port_target_default = "COM7" 
ip_default = "192.168.1.1" 
port_chaser_default = "USB0"

user = "pi"
password = "ermespi"

input_names = {'command_release': command_release_default, 'port_release': port_release_default, 
               'port_target': port_target_default, 'ip': ip_default, 'port_chaser': port_chaser_default}

command_high_lvl = "putty.exe -ssh pi@%s -pw ermespi -m C:\cmd\command_high.txt"
command_mid_lvl = "putty.exe -ssh pi@%s -pw ermespi -m C:\cmd\command_mid.txt"

def connection(command, port, name, baud, timeout=3): 
    with serial.Serial(port, baud, timeout=timeout) as connection:
        # time.sleep(0.1)
        if connection.isOpen():
            print(">> " + name + " connected to port " + str(connection.port) + "\n")
            try:
                connection.flushInput()
                connection.write(command)
                connection.close()
            except KeyboardInterrupt:
                print(">> error: keyboardInterrupt has been caught.")

def process_input():
    for i in range(1, len(sys.argv)):
        input_value = sys.argv[i].split(":=")
        globals()[input_value[0]] = input_value[1]
    
    for var in input_names.keys():
        if var not in globals():
            globals()[var] = input_names[var] 

    return command_release, port_release, port_target, ip, port_chaser

def launch_chaser(ip):
    cmd_high = command_high_lvl % str(ip)
    cmd_mid = command_mid_lvl % str(ip)
    os.system('cmd /k "' + cmd_mid + " && " + cmd_high + '"') 


if __name__ == "__main__":
    # ./script.py command_release:=value port_release:=value port_target:=value ip:=value port_chaser:=value

    print("experiment start\n")

    # process input (port, command, etc.)
    command_release, port_release, port_target, ip, port_chaser = process_input()
    command_release += '\n'
    command_release = command_release.encode('utf-8')

    # chaser start
    launch_chaser(ip)

    # release start
    # connection(command_release, port_release, "release", baud_release)

    # target start
    # connection(target_command, port_target, "target", baud_target)
