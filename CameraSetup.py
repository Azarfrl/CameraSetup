import cv2 as cv,threading,time,numpy as np,serial
from picamera2 import Picamera2

class Cam:
    def __init__(s):
        s.p=Picamera2()
        s.p.configure(s.p.create_video_configuration(main={"size":(640,480),"format":"RGB888"}))
        s.p.start()
        s.f=None
        s.l=threading.Lock()
        s.r=0
        s.fc=0
        s.t=time.time()
        s.src=np.float32([[100,180],[540,180],[620,450],[20,450]])
        s.dst=np.float32([[0,0],[320,0],[320,240],[0,240]])
        s.M=cv.getPerspectiveTransform(s.src,s.dst)

    def start(s):
        s.r=1
        threading.Thread(target=s.update,daemon=True).start()

    def update(s):
        while s.r:
            f=cv.cvtColor(s.p.capture_array(),cv.COLOR_RGB2BGR)
            with s.l:s.f=f
            s.fc+=1
            if time.time()-s.t>=1:
                s.fps=s.fc
                s.fc=0
                s.t=time.time()

    def read(s):
        with s.l:return None if s.f is None else s.f.copy()

    def stop(s):
        s.r=0
        s.p.stop()
        s.p.close()

class PID:
    def __init__(s,kp=.9,ki=.02,kd=.5,sp=160):
        s.kp,s.ki,s.kd,s.sp=kp,ki,kd,sp
        s.p=0
        s.i=0
        s.t=time.time()

    def c(s,v):
        e=s.sp-v
        n=time.time()
        d=n-s.t
        if d>0:
            s.i+=e*d
            der=(e-s.p)/d
        else:der=0
        o=s.kp*e+s.ki*s.i+s.kd*der
        s.p=e
        s.t=n
        return o

try:
    ser=serial.Serial('/dev/ttyACM0',115200,timeout=0,write_timeout=0)
    time.sleep(2)
    print("Arduino Connected")
except:
    ser=None
    print("No Arduino")

cam=Cam()
cam.start()

pid=PID()

base=40
maxs=35
lasts=time.time()

while 1:
    f=cam.read()
    if f is None:continue
    b=cv.warpPerspective(f,cam.M,(320,240))
    g=cv.cvtColor(b,cv.COLOR_BGR2GRAY)
    _,bin=cv.threshold(g,120,255,cv.THRESH_BINARY)
    bin=cv.dilate(bin,None,iterations=1)
    
    roi=bin[120:240,:]
    c,_=cv.findContours(roi,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    cen=[]

    if c:
        c=sorted(c,key=cv.contourArea,reverse=1)[:2]
        for x in c:
            if cv.contourArea(x)<300:continue
            xx,y,w,h=cv.boundingRect(x)
            y+=120
            cx=xx+w//2
            cen.append(cx)
            cv.rectangle(b,(xx,y),(xx+w,y+h),(0,255,0),2)
            cv.circle(b,(cx,y+h//2),5,(0,255,0),-1)

    if len(cen)==2:
        mid=(cen[0]+cen[1])//2
        out=pid.c(mid)
        cv.line(b,(mid,0),(mid,240),(255,0,255),2)

    elif len(cen)==1:
        mid=cen[0]
        out=pid.c(mid)

    else:
        mid=160
        out=pid.c(160)

    s=max(min(out,maxs),-maxs)
    l=base-s
    r=base+s

    if abs(s)<8:
        l+=8
        r+=8

    l=max(min(int(l),100),0)
    r=max(min(int(r),100),0)

    if ser and time.time()-lasts>.03:
        ser.write(f"L:{l} R:{r}\n".encode())
        lasts=time.time()

    cv.putText(f,f"L:{l} R:{r}",(10,40),0,.8,(0,255,0),2)
    cv.putText(f,f"FPS:{cam.fps}",(10,80),0,.8,(0,255,255),2)
    p=cam.src.astype(np.int32)
    cv.polylines(f,[p.reshape((-1,1,2))],1,(255,0,0),2)
    cv.imshow("Raw",f)
    cv.imshow("Birds",b)
    cv.imshow("Binary",bin)

    if cv.waitKey(1)&0xFF==ord('q'):break

cam.stop()

if ser:
    ser.write(b"L:0 R:0\n")
    ser.close()

cv.destroyAllWindows()
