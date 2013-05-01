#!/usr/bin/env python
# Copyright (c) 2013 Federico Ruiz Ugalde
# Author: Federico Ruiz-Ugalde <memeruiz at gmail dot com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.


import serial,pty,os,time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.flushInput()
master,slave=pty.openpty()
print os.ttyname(slave)
raw_input()

MSP_SET_RAW_RC=200
MSP_RC=105
MSP_MOTOR=104
MSP_RAW_IMU=102
MSP_IDENT=100
MSP_STATUS=101
MSP_ATTITUDE=108
MSP_RC_TUNING=111
MSP_RAW_GPS=106
MSP_COMP_GPS=107
MSP_ALTITUDE=109
MSP_ACC_CALIBRATION=205
MSP_MAG_CALIBRATION=206
HEADER='$M<'

l_time=0

def send_cmd(cmd, data):
    print "Data", data
    if cmd==MSP_SET_RAW_RC:
        data_size=16
        data_raw=[]
        for i in xrange(len(data)):
            #print data[i] & 0xff
            data_raw+=[data[i] & 0xff]
            data_raw+=[(data[i] >> 8) & 0xff]
            data_raw_chr=reduce(lambda x,y:x+y,map(chr,data_raw))
    elif cmd==MSP_RC or cmd==MSP_MOTOR or cmd==MSP_STATUS or cmd==MSP_IDENT or cmd==MSP_ATTITUDE or cmd==MSP_RC_TUNING or cmd==MSP_RAW_GPS or cmd==MSP_COMP_GPS or cmd==MSP_ALTITUDE:
        data_size=0
        data_raw=[]
        data_raw_chr=''
    elif cmd==MSP_RAW_IMU:
        data_size=0
        data_raw=[]
        data_raw_chr=''
    else:
        print "Error: Unknown command "
    checksum=(reduce(lambda x,y:x^y,data_raw+[cmd]+[data_size]) & 0xff)
    #print HEADER
    #print chr(data_size)+chr(cmd)+data_raw_chr+chr(checksum)
    ser_data=HEADER+chr(data_size)+chr(cmd)+data_raw_chr+chr(checksum)
    print "Data: ", data
    print "data_raw: ", data_raw
    print "Checksum: ", hex(checksum)
    print "data hex", map(hex,data_raw)
    print "ser_data", map(str,ser_data)
    print "ser_data_hex", map(ord,ser_data)
    ser.write(ser_data)

S_END=0
S_HEADER=1
S_SIZE=2
S_CMD=3
S_DATA=4
S_CHECKSUM=5
S_ERROR=6

def parse_input():
    data_to_read=1
    state=S_HEADER
    checksum=0
    input_packet=''
    while (state!=S_END) and (state!=S_ERROR):
        #print "Data to read", data_to_read
        c=ser.read(data_to_read)
        input_packet+=c
        #print "c: ", map(str,c), map(ord,c)
        if state==S_HEADER:
            if c=='$':
                c=ser.read(2)
                input_packet+=c
                #print "c: ", map(str,c), map(ord,c)
                if c=='M>':
                    #print "header received"
                    state=S_SIZE
                else:
                    print "Error in header1"
                    state=S_ERROR
            else:
                print "Error: No header received"
                state=S_ERROR
        elif state==S_SIZE:
            data_size=c
            checksum^=ord(data_size)
            state=S_CMD
            #print "Data size: ", ord(data_size)
        elif state==S_CMD:
            cmd=c
            checksum^=ord(cmd)
            if ord(data_size)==0:
                data_to_read=1
                state=S_CHECKSUM
            else:
                data_to_read=ord(data_size)
                state=S_DATA
            print "Command: ", ord(cmd)
        elif state==S_DATA:
            data_raw=c
            print "Data received", map(ord,data_raw)
            checksum^=reduce(lambda x,y:x^y, map(ord,data_raw))
            state=S_CHECKSUM
            data_to_read=1
        elif state==S_CHECKSUM:
            #print "Calculated checksum: ", checksum, " Received checksum: ", ord(c)
            if checksum!=ord(c):
                print "Error in checksum"
                state=S_ERROR
            else:
                print "Checksum is right"
                state=S_END
    if state==S_END:
        error=0
        if ord(cmd)==MSP_RC:
            #for i in xrange(len(data_raw)/2):
            #    print ord(data_raw[2*i]), ord(data_raw[2*i+1])
            #    print (ord(data_raw[2*i])<<0) | (ord(data_raw[2*i+1]) << 8)
            data=[(ord(data_raw[2*i])) | (ord(data_raw[2*i+1])<<8) for i in xrange(len(data_raw)/2)]
            print "Data: ", data
        elif ord(cmd)==MSP_RAW_IMU:
            data=[(ord(data_raw[2*i])) | (ord(data_raw[2*i+1])<<8) for i in xrange(len(data_raw)/2)]
            print "Data: ", data
        elif ord(cmd)==MSP_SET_RAW_RC:
            data=[]
        else:
            data=[]
        os.write(master,input_packet)
    else: #error
        print "input_packet: ", input_packet, map(ord, input_packet)
        data=[]
        error=1
        ser.flushInput()

    return((error,data))



while True:
    l_time+=1
    ser.flushInput()
    # raw_input()
    # print
    # print "Time: ", time
    # print "Command: ", MSP_SET_RAW_RC
    # data=[1500]*8
    # send_cmd(MSP_SET_RAW_RC, data)
    # data_in=ser.read(40)
    # print "Response"
    # print map(str,data_in)
    # print map(ord,data_in)

    # raw_input()
    # print
    # print "Reading RC controls"
    # print "Command: ", MSP_RC
    # send_cmd(MSP_RC,[])
    # data_in=ser.read(40)
    # print "Response"
    # print map(str,data_in)
    # print map(ord,data_in)

    #raw_input()
    print
    print "Reading Sensors"
    print "Command: ", MSP_RAW_IMU
    send_cmd(MSP_RAW_IMU,[])
    error,sensor_data=parse_input()
    print "Error: ", error
    print "Sensor data: ", sensor_data

    #raw_input()
    print
    print "Commanding copter"
    print "Command: ", MSP_SET_RAW_RC
    data=[1200]*8
    send_cmd(MSP_SET_RAW_RC,data)
    error,cmd_resp=parse_input()
    print "Error: ", error
    print "Command response: ", cmd_resp

    print
    print "Reading commanded RC"
    send_cmd(MSP_RC,[])
    error,cmded_data=parse_input()
    print "Error: ", error
    print "Commanded data: ", cmded_data

    print
    print "Reading motor"
    send_cmd(MSP_MOTOR,[])
    error,motor=parse_input()
    print "Error: ", error
    print "Data: ", motor

    print
    print "Reading a lot of stuff"
    send_cmd(MSP_IDENT,[])
    error,data=parse_input()
    print "Error: ", error
    print "Data: ", data
    send_cmd(MSP_STATUS,[])
    error,data=parse_input()
    print "Error: ", error
    print "Data: ", data
    send_cmd(MSP_ATTITUDE,[])
    error,data=parse_input()
    print "Error: ", error
    print "Data: ", data
    send_cmd(MSP_RC_TUNING,[])
    error,data=parse_input()
    print "Error: ", error
    print "Data: ", data
    send_cmd(MSP_ALTITUDE,[])
    error,data=parse_input()
    print "Error: ", error
    print "Data: ", data

    if error==1:
        pass
        #break
    time.sleep(0.1)
