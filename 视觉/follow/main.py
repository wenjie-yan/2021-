import sensor, image, time

from pid import PID
from pyb import Servo

pan_servo=Servo(1)
tilt_servo=Servo(2)

pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)

red_threshold  = (19, 63, 31, 73, -23, 60)
#红激光阈值：(44, 100, 3, 42, -5, 28)
pan_pid = PID(p=0.30, i=0.0, d=0,imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.07, i=0.00, d=0,imax=10) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_vflip(True)                                                         # 垂直方向翻转
sensor.set_hmirror(True)                                                    # 水平方向翻转
clock = time.clock() # Tracks FPS.

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

# 定义扫描参数
SCAN_LEFT = 20
SCAN_RIGHT = 160

pan_servo.angle(0,5000)

last_in_else = False  # 记录上次是否进入了else块

while(True):
    print(pan_servo.angle())
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
#    img.draw_line(80, 0, 80, 10,(0,0,255))
#    img.draw_line(0, 60, 10, 60,(0,0,255))
    blobs = img.find_blobs([red_threshold])
    if blobs:
        #扫描标志位
        max_blob = find_max(blobs)
        if (max_blob.area()<=20):
            continue
        pan_error =160/2-max_blob.cx()
        tilt_error  =120/2-max_blob.cy()

#        print("pan_error: ", pan_error)
#        print("tilt_error: ", tilt_error)

        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)/5
        tilt_output=tilt_pid.get_pid(tilt_error,1)
#        print("pan_output",pan_output)
#        print("tilt_output",tilt_output)
        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)
        last_in_else = False  # 重置标志位为False
    else:  # 如果未找到色块，开始水平扫描
        if not last_in_else:  # 如果上次不是进入了else块，执行以下代码
            tilt_servo.angle(90,10000)
            pan_servo.angle(0,10000)
            last_in_else = True  # 设置标志位为True
        if pan_servo.angle() < SCAN_LEFT:
            pan_servo.angle(180,5000)
        if pan_servo.angle() > SCAN_RIGHT:
            pan_servo.angle(0,5000)
