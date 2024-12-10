#!/usr/bin/env python3


import rospy
import json
import websocket
import time
from std_msgs.msg import Float64MultiArray


# ip = "10.5.32.11"  # "10.5.32.139"
ip = "10.5.32.139"
port = 5000

save_list = []

print("1: starting up the sensor")

rospy.init_node('xela_sensor')
print("2: starting up the sensor")
xela1_pub = rospy.Publisher('/xela1_data', Float64MultiArray, queue_size = 1)
print("3: starting up the sensor")
rate = rospy.Rate(100) # 100hz
print("4: starting up the sensor")
time.sleep(5)
print("5: starting up the sensor")

def publisher(xela_pub, data):
    xela_msg = Float64MultiArray()
    xela_msg.data = data
    xela_pub.publish(xela_msg)

def on_message(wsapp, message):

    data = json.loads(message)
    sensor1 = data['1']['calibrated']
    txls = int(len(sensor1)/3)
    data1_row = []
    
    for i in range(txls):

        x = sensor1[i*3]
        y = sensor1[i*3+1]
        z = sensor1[i*3+2]
        data1_row.append(x)
        data1_row.append(y)
        data1_row.append(z)

    publisher(xela1_pub, data1_row)
    rate.sleep()

print("6: starting up the sensor")
wsapp = websocket.WebSocketApp("ws://{}:{}".format(ip,port), on_message=on_message)
wsapp.run_forever()