#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import torch
import torch.nn as nn
import pickle
import torch.nn.functional as F

data_stream = []
hidden_size = [256]
num_classes = 15
input_size = 1147
block_label = ""

# model_path = '/home/jetbot/FedLBEClient/FedLBE/Client/model.pt'
model_path = '/home/jetbot/numpy_arrays/Two/modelFF.pt'
#model_path = '/home/jetbot/numpy_arrays/Three/modelFF.pt'
# model_path = '/home/jetbot/numpy_arrays/modelFF.pt'
model_state_dict = torch.load(model_path) 
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# input_size = len(msg.ranges)

class Test_Net(nn.Module):
    def __init__(self):
        super(Test_Net, self).__init__()
        self.fc1 = nn.Linear(1147,256)
        self.fc2 = nn.Linear(256,15)
    def forward(self, x):
        x = F.relu(self.fc1(x))
        output = self.fc2(x)
        return output

# Load the model
model_state_dict = torch.load(model_path,map_location=device)
model_test = Test_Net().to(device)
model_test.load_state_dict(model_state_dict)
model_test.eval()  # Set the model to evaluation mode

# Load the label_mapping dictionary
with open('/home/jetbot/numpy_arrays/label_mapping.pkl', 'rb') as f:
    label_mapping = pickle.load(f)

print('Starting Inference Process')
def callback(msg):
    global data_stream
    global block_label
    np_array = np.asarray(msg.ranges)
    np_array[np_array == np.inf] = 2
    np_array[np_array > 0.4] = 2
    np_array = torch.tensor(np_array).float().to(device)
    np_array = np_array.unsqueeze(0)



    with torch.no_grad():
        outputs = model_test(np_array)
        _, predicted = torch.max(outputs.data, dim = 1)
    predicted_labels = [label_mapping[p.item()] for p in predicted]


    data_stream.append(np_array)
    # print(predicted_labels)
    block_label = listToString(predicted_labels)
    # print(type(block_label)

    # print(len(msg.ranges))

def main():
    rospy.init_node('infer_model')
    block_pub = rospy.Publisher('/block_type', String,queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    # rospy.spin()

    while not rospy.is_shutdown():
       block_pub.publish(block_label)
    #    x=2

def listToString(s):
     
    # initialize an empty string
    str1 = ""
 
    # traverse in the string
    for ele in s:
        str1 += ele
 
    # return string
    return str1

if __name__ == '__main__':
    main()

