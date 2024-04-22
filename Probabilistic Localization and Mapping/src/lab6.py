#! /usr/bin/env python3

# importing libraries and dependencies
from hashlib import new
from turtle import distance, position
from unicodedata import name
import rospy
import math
import random
import string
import numpy as np
from std_msgs.msg import String
import matplotlib.pyplot as mpl


# Function Definition - Publisher Node
def publisher_node(inputParam) :
    publisher = rospy.Publisher('robot',String,queue_size=1)
    rate = rospy.Rate(1)
    publisher.publish(inputParam)
    subscriber_node()

# Function Definition - Subscriber node is called
def subscriber_node() :
    rospy.Subscriber('robot',String,callback)

# Function Definition - Callback function 
def callback(subscriber_data) :
    startPositionX = 10
    startPositionY = 10
    global probabilityMatrix
    probabilityMatrix = np.zeros([20,20])
    directionChar = subscriber_data.data[0]
    translationValue = subscriber_data.data[1]
    translationValue = int(translationValue)
    possibleSteps = []
    probabilityMatrix[startPositionX][startPositionY] = 1
    if directionChar == 'l' or directionChar == 'L':
        if translationValue < 10 and translationValue > 0 :
            possibleStepCount = 0
            while possibleStepCount < (2* translationValue + 1) and possibleStepCount < 10:
                possibleSteps.append(startPositionY - possibleStepCount)
                possibleStepCount = possibleStepCount + 1
            print(possibleSteps)
            for currentStep in possibleSteps:
                possibleStepCount = 0
                while(possibleStepCount<3 and currentStep>=(startPositionY-translationValue)) :
                    if probabilityMatrix[startPositionX][currentStep] > 0:
                        if possibleStepCount == 0:
                            probabilityMatrix[startPositionX][currentStep] = probabilityMatrix[startPositionX][currentStep]* 0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[startPositionX][currentStep-1] = probabilityMatrix[startPositionX][currentStep-1]* 0.6
                        else :
                            probabilityMatrix[startPositionX][currentStep-2] = probabilityMatrix[startPositionX][currentStep-2]* 0.2
                    else :
                        if possibleStepCount == 0:
                            probabilityMatrix[startPositionX][currentStep] =  0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[startPositionX][currentStep-1] = 0.6
                        else :
                            probabilityMatrix[startPositionX][currentStep-2] =  0.2
                    possibleStepCount = possibleStepCount + 1

    elif directionChar == 'r' or directionChar == 'R':
        if translationValue < 10 and translationValue < 20 :
            possibleStepCount = 0
            while possibleStepCount < (2* translationValue + 1) and possibleStepCount < 10:
                possibleSteps.append(startPositionX + possibleStepCount)
                possibleStepCount = possibleStepCount + 1
            print(possibleSteps)
            for currentStep in possibleSteps:
                possibleStepCount = 0
                while(possibleStepCount<3 and currentStep>=(startPositionY-translationValue)) :
                    if probabilityMatrix[startPositionX][currentStep] > 0:
                        if possibleStepCount == 0:
                            probabilityMatrix[startPositionX][currentStep] = probabilityMatrix[startPositionX][currentStep]* 0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[startPositionX][currentStep+1] = probabilityMatrix[startPositionX][currentStep+1]* 0.6
                        else :
                            probabilityMatrix[startPositionX][currentStep+2] = probabilityMatrix[startPositionX][currentStep+2]* 0.2
                    else :
                        if possibleStepCount == 0:
                            probabilityMatrix[startPositionX][currentStep] =  0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[startPositionX][currentStep+1] = 0.6
                        else :
                            probabilityMatrix[startPositionX][currentStep+2] =  0.2
                    possibleStepCount = possibleStepCount + 1
    elif directionChar == 'u' or directionChar == 'U':
        if translationValue < 10 and translationValue < 20 :
            possibleStepCount = 0
            while possibleStepCount < (2* translationValue + 1) and possibleStepCount < 10:
                possibleSteps.append(startPositionX + possibleStepCount)
                possibleStepCount = possibleStepCount + 1
            print(possibleSteps)
            for currentStep in possibleSteps:
                possibleStepCount = 0
                while(possibleStepCount<3 and currentStep>=(startPositionX+translationValue)) :
                    if probabilityMatrix[currentStep][startPositionY] > 0:
                        if possibleStepCount == 0:
                            probabilityMatrix[currentStep][startPositionY] = probabilityMatrix[currentStep][startPositionY]* 0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[currentStep+1][startPositionY] = probabilityMatrix[currentStep+1][startPositionY]* 0.6
                        else :
                            probabilityMatrix[currentStep+2][startPositionY] = probabilityMatrix[currentStep+2][startPositionY]* 0.2
                    else :
                        if possibleStepCount == 0:
                            probabilityMatrix[currentStep][startPositionY] =  0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[currentStep+1][startPositionY] = 0.6
                        else :
                            probabilityMatrix[currentStep+2][startPositionY] =  0.2
                    possibleStepCount = possibleStepCount + 1
    elif directionChar == 'd' or directionChar == 'D':
        if translationValue < 10 and translationValue < 20 :
            possibleStepCount = 0
            while possibleStepCount < (2* translationValue + 1) and possibleStepCount < 10:
                possibleSteps.append(startPositionX + possibleStepCount)
                possibleStepCount = possibleStepCount + 1
            print(possibleSteps)
            for currentStep in possibleSteps:
                possibleStepCount = 0
                while(possibleStepCount<3 and currentStep>=(startPositionX-translationValue)) :
                    if probabilityMatrix[currentStep][startPositionY] > 0:
                        if possibleStepCount == 0:
                            probabilityMatrix[currentStep][startPositionY] = probabilityMatrix[currentStep][startPositionY]* 0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[currentStep-1][startPositionY] = probabilityMatrix[currentStep-1][startPositionY]* 0.6
                        else :
                            probabilityMatrix[currentStep-2][startPositionY] = probabilityMatrix[currentStep-2][startPositionY]* 0.2
                    else :
                        if possibleStepCount == 0:
                            probabilityMatrix[currentStep][startPositionY] =  0.2
                        elif possibleStepCount == 1 :
                            probabilityMatrix[currentStep-1][startPositionY] = 0.6
                        else :
                            probabilityMatrix[currentStep-2][startPositionY] =  0.2
                    possibleStepCount = possibleStepCount + 1
    else :
        print('invalid direction')
    
    

    

#Print and recieve input values
if __name__ == '__main__':
    rospy.init_node('lab6',anonymous=True)
    input = input("Enter the Input in Character + Integer Format: ")
    while not rospy.is_shutdown():
        publisher_node(input)
    plot = mpl.contourf(probabilityMatrix, cmap=mpl.cm.rainbow)
    mpl.colorbar()  
    mpl.show()
