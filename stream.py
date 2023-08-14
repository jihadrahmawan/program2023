#pip3 install flask
#pip3 install imutils
#pip3 install opencv-contrib-python

from imutils.video import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
import numpy as np
from elements.yolo import OBJ_DETECTION
import threading
import datetime
import imutils
import time
import cv2

Object_classes = ['b', 'a']

Object_colors = list(np.random.rand(80,3)*255)
Object_detector = OBJ_DETECTION('weights/best.pt', Object_classes)

outputFrame = None
lock = threading.Lock()

app = Flask(__name__)

vs = cv2.VideoCapture(0)

time.sleep(2.0)
k = 1
@app.route("/")
def index():
	
	return render_template("index.html")

def detect_motion():
	
	global vs, outputFrame, lock,Object_colors,Object_detector,k
	
	while True:
		
		suc, frame = vs.read()
		#frame = imutils.resize(frame, width=400)
		if suc:
		  objs = Object_detector.detect(frame)
		  for obj in objs:
		    label = obj['label']
		    if label == 'a' or label =='b':
		      score = obj['score']
		      [(xmin,ymin),(xmax,ymax)] = obj['bbox']
		      color = (0,102,255)
		      frame = cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2) 
		      frame = cv2.putText(frame, f'{label} [{str(score)}]', (xmin,ymin-20), cv2.FONT_HERSHEY_SIMPLEX , 1, color, 1, cv2.LINE_AA)
		    else:
		      continue

		
		timestamp = datetime.datetime.now()
		
		with lock:
			outputFrame = frame.copy()

def generate():
	# grab global references to the output frame and lock variables
	global outputFrame, lock
	# loop over frames from the output stream
	while True:
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			if outputFrame is None:
				continue
			# encode the frame in JPEG format
			(flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
			# ensure the frame was successfully encoded
			if not flag:
				continue
		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
			bytearray(encodedImage) + b'\r\n')

@app.route("/video_feed")
def video_feed():
	
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

# check to see if this is the main thread of execution
if __name__ == '__main__':
	
	t = threading.Thread(target=detect_motion)
	t.daemon = True
	t.start()
	# start the flask app
	app.run(host='0.0.0.0', port=8888, debug=True,
		threaded=True, use_reloader=False)
# release the video stream pointer
vs.release()
