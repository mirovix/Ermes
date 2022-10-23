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
import keyboard
from datetime import datetime

baud_target = 38400
baud_release = 9600

target_command = b"a"
dict_command_release = {'0': "SETUP", '1': "s1", '2': "s2", '3': "s3", '4': "s4"}

command_release_default = "SETUP"
port_release_default = "COM6"
port_target_default = "COM11" 
ip_default = "192.168.137.224" 
port_chaser_default = "USB0"

user = "pi"
password = "ermespi"

input_names = {'command_release': command_release_default, 'port_release': port_release_default, 
               'port_target': port_target_default, 'ip': ip_default, 'port_chaser': port_chaser_default}

command_mid_lvl = "cd ermes_catkin_pi/ && source ~/ermes_catkin_pi/devel/setup.bash && rosrun mid_lvl mid_lvl %s >> log_mid_%s.txt"
end_command = "putty.exe -ssh pi@%s -pw ermespi -m C:\cmd\command_kill.txt"

def connection(command, port, name, baud, timeout=3, print_out=False): 
    with serial.Serial(port, baud, timeout=timeout) as connection:
        # time.sleep(0.1)
        if connection.isOpen():
            print(">> " + name + " connected to port " + str(connection.port) + "\n")
            try:
                connection.write(command)
                if print_out is True:
                    while connection.inWaiting()==0: pass
                    if connection.inWaiting()>0: 
                        answer = connection.readline()
                        print(">> slider output : " + str(answer))
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


def ssh_conn(ip, command):
    ssh = paramiko.SSHClient()
    # Load SSH host keys.
    ssh.load_system_host_keys()
    # Add SSH host key when missing.
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    total_attempts = 1
    for attempt in range(total_attempts):
        try:
            print(">> attempt to connect: %s" % attempt)
            # Connect to router using username/password authentication.
            ssh.connect(ip, 
                        username=user, 
                        password=password,
                        look_for_keys=False)
            # Run command.
            ssh.exec_command(command)
            # Read output from command.
            # output = ssh_stdout.readlines()
            # Close connection.
            ssh.close()
            # return output

        except Exception as error_message:
            print(">> unable to connect")
            print(error_message)

def ssh_conn_kill(ip, command):
    cmd = command % ip
    print(cmd)
    os.system('cmd /k "' + cmd + '"') 

if __name__ == "__main__":
    # ./script.py command_release:=value port_release:=value port_target:=value ip:=value port_chaser:=value

    print(">> experiment start\n")

    print(">> waiting... \n")
    time.sleep(9)

    # process input (port, command, etc.)
    command_release, port_release, port_target, ip, port_chaser = process_input()
    # command_release = dict_command_release[command_release] + "\r\n"
    # command_release = command_release.encode('utf-8')

    # chaser start
    now = datetime.now()
    dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
    cmd = command_mid_lvl % (str(port_chaser), str(dt_string))
    ssh_conn(ip, cmd)
    print(">> chaser start\n")

    # release start
    # connection(command_release, port_release, "release", baud_release, print_out=True)
    # print(">> release start\n")

    # target start
    connection(target_command, port_target, "target", baud_target)
    print(">> target start\n")

    keyboard.wait("q")

    # end the mid_lvl
    ssh_conn_kill(ip, end_command)


