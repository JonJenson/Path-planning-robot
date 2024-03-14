from time import sleep
import cv2
import numpy as np
import math
import os
import RPi.GPIO as GPIO
from gpiozero import Robot

GPIO.setmode(GPIO.BCM)

a = 1
b = 0.9
c = 0.8
d = 0.7
e = 0.6
f = 0.5
g = 0.4

testmode = 1
m1_1 = 6
m1_2 = 5

m2_1 = 18
m2_2 = 23

GPIO.setup(m1_1, GPIO.OUT)
GPIO.setup(m1_2, GPIO.OUT)
GPIO.setup(m2_1, GPIO.OUT)
GPIO.setup(m2_2, GPIO.OUT)

r = Robot(left=(m1_1, m1_2), right=(m2_1, m2_2))

def forward():
    m1_speed = 0.8
    m2_speed = a
    r.value = (m1_speed, m2_speed)

def backward():
    r.reverse()

def right():
    r.right(speed=1)
    print("Going right")
    sleep(0.6)
    forward()

def left():
    r.left(speed=1)
    print("Going left")
    sleep(0.6)
    forward()

def stop():
    m1_speed = 0.0
    m2_speed = 0.0
    r.value = (m1_speed, m2_speed)
    print('going off')

def calc_dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def getChunks(l, n):
    """Yield successive n-sized chunks from l."""
    a = []
    for i in range(0, len(l), n):
        a.append(l[i:i + n])
    return a

cap = cv2.VideoCapture(0)

try:
    if not os.path.exists('data'):
        os.makedirs('data')
except OSError:
    print('Error: Creating directory of data')

stepsize = 5
currentFrame = 0

if testmode == 2:
    with open("./data/imagedetails.txt", 'a') as F:
        F.write("\n\nWew Test \n")

while True:
    _, img = cap.read()
    frame = cv2.flip(img, 1)

    if testmode == 1:
        name = './data/frame' + str(currentFrame) + '.jpg'
        print('Creating...' + name)

    blur = cv2.bilateralFilter(frame, 9, 40, 40)
    edges = cv2.Canny(blur, 50, 100)
    img_h = frame.shape[0] - 1
    img_w = frame.shape[1] - 1
    EdgeArray = []

    for j in range(0, img_w, stepsize):
        pixel = (j, 0)
        for i in range(img_h - 5, 0, -1):
            if edges.item(i, j) == 255:
                pixel = (j, i)
                break
        EdgeArray.append(pixel)

    for x in range(len(EdgeArray) - 1):
        cv2.line(frame, EdgeArray[x], EdgeArray[x + 1], (0, 255, 0), 1)

    for x in range(len(EdgeArray)):
        cv2.line(frame, (x * stepsize, img_h), EdgeArray[x], (0, 255, 0), 1)

    chunks = getChunks(EdgeArray, int(len(EdgeArray) / 3))
    max_dist = 0
    c = []

    for i in range(len(chunks) - 1):
        x_vals = []
        y_vals = []
        for (x, y) in chunks[i]:
            x_vals.append(x)
            y_vals.append(y)
        avg_x = int(np.average(x_vals))
        avg_y = int(np.average(y_vals))
        c.append([avg_y, avg_x])
        cv2.line(frame, (320, 480), (avg_x, avg_y), (255, 0, 0), 2)

    forwardEdge = c[1]
    cv2.line(frame, (320, 480), (forwardEdge[1], forwardEdge[0]), (0, 255, 0), 3)
    cv2.imwrite(name, frame)

    y = min(c)
    print(y)

    if testmode == 1:
        cv2.imshow("frame", frame)
        cv2.imshow("canny", edges)
        cv2.imshow("result", img)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    if cv2.waitKey(1) & 0xFF == ord("q"):
        stop()
        GPIO.cleanup()
        break

cv2.destroyAllWindows()
cap.release()
