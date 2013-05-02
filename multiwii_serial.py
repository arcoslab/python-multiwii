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
from arcospyu.dprint import eprint

master,slave=pty.openpty()
print os.ttyname(slave)
print "Create a symbolic link in /dev/ to the above file and use this symlink to connect with MultiWiiConf to help in debugging. Press enter when ready"
#raw_input()

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

import struct

def toraw_data(data, word_size):
    #print "Data", data
    return(struct.pack(word_size, *data))
    # data_raw=[]
    # for i in xrange(len(data)):
    #     for j in xrange(word_size):
    #         data_raw+=[((data[i] >> 8*j) & 0xff)]
    # return(data_raw)


def unraw_data(raw_data, word_size):
    #print "Size", len(raw_data)/word_size
    #print word_size
    #print "Raw data", raw_data
    return(struct.unpack(word_size, raw_data))
#    data=[]
#    for i in xrange(len(raw_data)/word_size):
#        x=0
#        for j in xrange(word_size):
#            x+=(ord(raw_data[i*word_size+j]))<<8*j
#        data.append(x)
#    return(data)

class MultiwiiCopter(object):
    MSP_SET_RAW_RC=200
    MSP_RC=105
    MSP_MOTOR=104
    MSP_RAW_IMU=102
    MSP_IDENT=100
    MSP_STATUS=101
    MSP_ATTITUDE=108
    MSP_RC_TUNING=111
    MSP_CONTROL=120
    MSP_RAW_GPS=106
    MSP_COMP_GPS=107
    MSP_ALTITUDE=109
    MSP_ACC_CALIBRATION=205
    MSP_MAG_CALIBRATION=206
    HEADER='$M<'
    S_END=0
    S_HEADER=1
    S_SIZE=2
    S_CMD=3
    S_DATA=4
    S_CHECKSUM=5
    S_ERROR=6
    #msp_dict: cmd: [cmd_data_size, response_data_size, word_size]
    msp_dict={MSP_SET_RAW_RC: [16, 0, '='+'h'*8],
              MSP_RC: [0, 16, '', '='+'h'*8],
              MSP_ATTITUDE: [0, 8, '', '='+'h'*4],
              MSP_ALTITUDE: [0, 6, '', '=ih', [0.01, 1.]],
              MSP_STATUS: [0, 11, '', '='+'hhhIB'],
              MSP_CONTROL: [16, 14, '='+'h'*8, '='+'h'*7, [0.1, 0.1, -1., 1., 1., 1., 1.]]}

    def __init__(self, serialport="/dev/ttyUSB0", speed=115200):
        self.ser = serial.Serial(serialport, speed, timeout=1)
        self.ser.flushInput()

    def __del__(self):
        self.ser.close()

    def send_serial(self, message_type, data):
        if self.msp_dict[message_type][0]==0:
            data_raw_chr=''
        else:
            data_raw_chr=toraw_data(data, self.msp_dict[message_type][2])
        #print "Data raw chr", data_raw_chr
        data_raw=list(struct.unpack('B'*self.msp_dict[message_type][0], data_raw_chr))
        #print "Data raw", data_raw
        cmd_data_size=self.msp_dict[message_type][0]
        #print "Cmd data size", cmd_data_size
        if len(data_raw) != cmd_data_size:
            eprint("Error: data size incorrect for this message type. MSG type: ", message_type, " data size:", len(data), " expected data size:", cmd_data_size)
            return(-1)
        if message_type not in self.msp_dict:
            eprint("Invalid message: ", message_type)
            return(-1)
        #data_raw=[]
        # for i in data:
        #     data_raw+=[data[i] & 0xff, (data[i] >> 8) & 0xff]
        if len(data_raw)>0:
            data_raw_chr=reduce(lambda x,y:x+y, map(chr, data_raw))
        else:
            data_raw_chr=''
        checksum=(reduce(lambda x,y:x^y,data_raw+[message_type]+[cmd_data_size]) & 0xff)
        ser_data=self.HEADER+chr(cmd_data_size)+chr(message_type)+data_raw_chr+chr(checksum)
        #print "Data: ", data
        #print "data_raw: ", data_raw
        #print "Checksum: ", hex(checksum)
        #print "data hex", map(hex,data_raw)
        #print "ser_data", map(str,ser_data)
        #print "ser_data_hex", map(ord,ser_data)
        self.ser.write(ser_data)

    def recv_serial(self):
        data_to_read=1
        state=self.S_HEADER
        checksum=0
        input_packet=''
        while (state!=self.S_END) and (state!=self.S_ERROR):
            #print "in waiting", self.ser.inWaiting()
        #print "Data to read", data_to_read
            c=self.ser.read(data_to_read)
            input_packet+=c
            #print "c: ", map(str,c), map(ord,c)
            if state==self.S_HEADER:
                if c=='$':
                    c=self.ser.read(2)
                    input_packet+=c
                #print "c: ", map(str,c), map(ord,c)
                    if c=='M>':
                    #print "header received"
                        state=self.S_SIZE
                    else:
                        print "Error in header1"
                        state=self.S_ERROR
                else:
                    print "Error: No header received"
                    state=self.S_ERROR
            elif state==self.S_SIZE:
                data_size=c
                checksum^=ord(data_size)
                state=self.S_CMD
            #print "Data size: ", ord(data_size)
            elif state==self.S_CMD:
                cmd=c
                checksum^=ord(cmd)
                if ord(data_size)==0:
                    data_to_read=1
                    state=self.S_CHECKSUM
                    data_raw=''
                else:
                    data_to_read=ord(data_size)
                    state=self.S_DATA
                #print "Command: ", ord(cmd)
            elif state==self.S_DATA:
                data_raw=c
                #print "Data received", map(ord,data_raw)
                checksum^=reduce(lambda x,y:x^y, map(ord,data_raw))
                state=self.S_CHECKSUM
                data_to_read=1
            elif state==self.S_CHECKSUM:
            #print "Calculated checksum: ", checksum, " Received checksum: ", ord(c)
                if checksum!=ord(c):
                    eprint("Error in checksum")
                    state=self.S_ERROR
                else:
                    #print "Checksum is right"
                    state=self.S_END
        if state==self.S_END:
            error=0
            if len(data_raw)!= self.msp_dict[ord(cmd)][1]:
                eprint("Error: incorrect response size! Got:", len(data_raw), " should be:", self.msp_dict[ord(cmd)][1])
                return((-1,[]))
            #print "Cmd", ord(cmd)
            #print "Raw data", data_raw, "test"
            if self.msp_dict[ord(cmd)][1]!=0:
                  data=unraw_data(data_raw, self.msp_dict[ord(cmd)][3])
                  if len(self.msp_dict[ord(cmd)])>=5:
                      data=[i*mult for i, mult in zip(self.msp_dict[ord(cmd)][4], data)]
            else:
                  data=[]
            #os.write(master,input_packet)
        else: #error
            eprint("input_packet: ", input_packet, map(ord, input_packet))
            data=[]
            cmd=chr(0)
            error=1
            self.ser.flushInput()
        return((error,[ord(cmd),data]))


    def rc_cmd(self, cmd):
        self.send_serial(self.MSP_SET_RAW_RC, cmd)
        error,cmd_resp=self.recv_serial()
        if error!=0:
            eprint("RC command didn't work!")
        return(error, cmd_resp)

    def control(self, cmd):
        self.send_serial(self.MSP_CONTROL, cmd)
        error,cmd_resp=self.recv_serial()
        if error!=0:
            eprint("CONTROL didn't work!")
        return(error, cmd_resp)

    def rc_read(self):
        self.send_serial(self.MSP_RC, [])
        error, cmd_resp=self.recv_serial()
        if error!=0:
            eprint("RC read didn't work!")
        return(error, cmd_resp)

    def get_attitude(self):
        self.send_serial(self.MSP_ATTITUDE, [])
        error, cmd_resp=self.recv_serial()
        if error!=0:
            eprint("RC attitude didn't work!")
        return(error, cmd_resp)

    def get_altitude(self):
        self.send_serial(self.MSP_ALTITUDE, [])
        error, cmd_resp=self.recv_serial()
        if error!=0:
            eprint("RC ALTITUDE didn't work!")
        return(error, cmd_resp)

    def get_status(self):
        self.send_serial(self.MSP_STATUS, [])
        error, cmd_resp=self.recv_serial()
        if error!=0:
            eprint("STATUS didn't work!")
        return(error, cmd_resp)

from numpy import array, pi, dot
from numpy.linalg import norm, inv
from arcospyu.kdl_helpers import kdl_helpers as kh
from arcospyu.robot_tools import robot_trans as rt
class Copter(object):
    G=9.81 #m/s^2
    G_vec=[0., 0., -G]
    G_frame=[[1., 0., 0., 0.],
             [0., 1., 0., 0.],
             [0., 0., 1., G],
             [0., 0., 0., 1.]]
    raw_acc_trans=array([[0., 1., 0.],
                   [-1., 0., 0.],
                   [0., 0., -1.]])
    att_trans=array([[1., 0., 0.],
                     [0., 1., 0.],
                     [0., 0., 1.]])
    def __init__(self, speed=115200):
        self.copter_serial=MultiwiiCopter(speed=speed)
        self.cmd=[1000]+[1500]*7
        self.attitude=array([0.]*3)
        self.acc_raw=array([0.]*3)
        self.acc=array([0.]*3)
        self.copter_control_freq=0.
        self.x=array([0.]*3)
        self.num_grav_measures=100
        self.grav_cal_const=1.
        self.delta_time=0.
        self.old_time=time()
        self.vel=array([0.]*3)

    def update_sensors(self):
        self.update_delta_time()
        #print "delta", self.delta_time
        error, resp=self.copter_serial.control(self.cmd)
        if error==0:
            if resp[1][-1]>0:
                self.copter_control_freq=1000000./resp[1][-1]
            self.attitude=dot(self.att_trans,array(resp[1][:3])*pi/180.)
            self.attitude_frame=kh.rpy_to_rot_matrix(self.attitude)
            #print "frame: ", self.attitude_frame
            #print "Att: ", self.attitude
            #print "G vec: ", self.G_vec
            #print "frame inv: ", inv(self.attitude_frame)
            self.acc_expected=dot(inv(self.attitude_frame),self.G_vec)
            #print "Expected acc", self.acc_expected,
            self.acc_raw=dot(self.raw_acc_trans,array(resp[1][3:6]))
            #self.acc_raw_mag_rot=kh.rpy_to_rot_matrix(array([0., 0., self.attitude[2]]))
            #self.acc_raw=dot(self.acc_raw_mag_rot, self.acc_raw)
            self.acc=self.acc_raw*self.grav_cal_const
            self.acc_comp=dot(self.attitude_frame,self.acc_expected-self.acc)
            #print "ACC_Comp", self.acc_comp, "Diff norm", norm(self.acc_comp)
            return(0)
        else:
            return(-1)

    def calc_grav(self):
        grav_measure=0.
        for i in xrange(self.num_grav_measures):
            #print "Norm", norm(self.acc_raw)
            while not (self.update_sensors()==0):
                print "No updates ready yet"
            grav_measure=grav_measure*(float(i)/(i+1.))+norm(self.acc_raw)*(1./(i+1.))
            #print "Grav", grav_measure
        #print "Grav measure", grav_measure
        self.grav_cal_const=self.G/grav_measure
        #print "Grav cal const", self.grav_cal_const
        #kh.my_diff(G_frame, 
        raw_input()

    def update_delta_time(self):
        t=time()
        self.delta_time=t-self.old_time
        self.old_time=t

    def update_vel(self):
        self.vel+=self.acc_comp*self.delta_time
        print "Vel:", self.vel, "Acc:", self.acc_comp,

    def update_pos(self):
        self.x+=self.vel*self.delta_time
        print "Pos", self.x

from time import time, sleep

def main():
    copter=Copter(speed=115200)
    first=True
    counter=0
    rc_cmd=[1230]*8
    copter.calc_grav()
    while True:
        cur_time=time()
        if not first:
            period=cur_time-old_time
            if counter==100:
                print "Period: ", period, "Freq: ", 1./period
                counter=0
            else:
                counter+=1
        first=False
        old_time=cur_time
        #l_time+=1
        #error, resp=copter.rc_cmd(rc_cmd)
        #print "RC cmd resp:", error, resp
        #error, resp=copter.rc_read()
        #print "RC read resp:", error, resp
        #error, resp=copter.get_attitude()
        #print "Attitude resp:", error, resp
        #error, resp=copter.get_altitude()
        #print "Altitude resp:", error, resp
        #error, resp=copter.get_status()
        #print "Status:", resp, "Freq: ", 1000000./resp[1][0]
        copter.update_sensors()
        copter.update_vel()
        copter.update_pos()
        #print "Freq:", copter.copter_control_freq, " Att:", copter.attitude, "acc:", copter.acc, "norm", norm(copter.acc)

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
    # print
    # print "Reading Sensors"
    # print "Command: ", MSP_RAW_IMU
    # send_cmd(MSP_RAW_IMU,[])
    # error,sensor_data=parse_input()
    # print "Error: ", error
    # print "Sensor data: ", sensor_data

    #raw_input()
        # print
        # print "Commanding copter"
        # print "Command: ", MSP_SET_RAW_RC
        # data=[1650]*8
        # send_cmd(MSP_SET_RAW_RC,data)
        # error,cmd_resp=parse_input()
        # print "Error: ", error
        # print "Command response: ", cmd_resp

        # print
        # print "Reading commanded RC"
        # send_cmd(MSP_RC,[])
        # error,cmded_data=parse_input()
        # print "Error: ", error
        # print "Commanded data: ", cmded_data

    # print
    # print "Reading motor"
    # send_cmd(MSP_MOTOR,[])
    # error,motor=parse_input()
    # print "Error: ", error
    # print "Data: ", motor

        #print
        #print "Reading a lot of stuff"
    # send_cmd(MSP_IDENT,[])
    # error,data=parse_input()
    # print "Error: ", error
    # print "Data: ", data
    # send_cmd(MSP_STATUS,[])
    # error,data=parse_input()
    # print "Error: ", error
    # print "Data: ", data
        #send_cmd(MSP_ATTITUDE,[])
        #error,data=parse_input()
        #print "Error: ", error
        #print "Data: ", data
        # send_cmd(MSP_RC_TUNING,[])
    # error,data=parse_input()
    # print "Error: ", error
    # print "Data: ", data
    # send_cmd(MSP_ALTITUDE,[])
    # error,data=parse_input()
    # print "Error: ", error
    # print "Data: ", data

        #break
        #time.sleep(0.01)
    

if __name__=="__main__":
    main()

