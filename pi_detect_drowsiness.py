# USAGE
# python pi_detect_drowsiness.py --cascade haarcascade_frontalface_default.xml --shape-predictor shape_predictor_68_face_landmarks.dat
# python pi_detect_drowsiness.py --cascade haarcascade_frontalface_default.xml --shape-predictor shape_predictor_68_face_landmarks.dat --alarm 1

# import the necessary packages
from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2
import serial


def send_str(ser, s):
	for i in range(len(s)):
		ser.write(s[i].encode())
		while (ser.read(1) != 'K'):
			time.sleep(1.0)
		print("K recived!")
	ser.write('\n')
	return 0;

def euclidean_dist(ptA, ptB):
	# compute and return the euclidean distance between the two
	# points
	return np.linalg.norm(ptA - ptB)

def eye_aspect_ratio(eye):
	# compute the euclidean distances between the two sets of
	# vertical eye landmarks (x, y)-coordinates
	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])

	# compute the euclidean distance between the horizontal
	# eye landmark (x, y)-coordinates
	C = dist.euclidean(eye[0], eye[3])

	# compute the eye aspect ratio
	ear = (A + B) / (2.0 * C)

	# return the eye aspect ratio
	return ear
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--cascade", required=True,
	help = "path to where the face cascade resides")
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
ap.add_argument("-w", "--webcam", type=int, default=0,
	help="index of webcam on system")
args = vars(ap.parse_args())
 
# define two constants, one for the eye aspect ratio to indicate
# blink and then a second constant for the number of consecutive
# frames the eye must be below the threshold
EYE_AR_THRESH = 0.3
EYE_AR_CONSEC_FRAMES = 1
EYE_AR_CONSEC_FRAMES_SLEEP = 56
send_counter = 0
ear = 0
ser = serial.Serial ("/dev/ttyS0")    #Open named port
ser.baudrate = 9600                     #Set baud rate to 9600

frame_count = 0
window = 300
blink_rates = []
#std = np.std(blink_rates)
blink_rate = 0
blink_rates.append(0.035)

blink_duration = 3
blink_durations = []
blink_durations.append(blink_duration)

head_positions = []
head_down_flag = False
head_up_flag = False
# initialize the frame counters and the total number of blinks
COUNTER = 0
counter_head = 0
TOTAL = 0

# load OpenCV's Haar cascade for face detection (which is faster than
# dlib's built-in HOG detector, but less accurate), then create the
# facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = cv2.CascadeClassifier(args["cascade"])
predictor = dlib.shape_predictor(args["shape_predictor"])

# grab the indexes of the facial landmarks for the left and
# right eye, respectively
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
frame_count = 0
blink_rates_av = 0
blink_durations_av = 0
# start the video stream thread
print("[INFO] starting video stream thread...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
time.sleep(1.0)

# loop over frames from the video stream
while True:
	if frame_count == 0:
		start = time.time()

	# grab the frame from the threaded video file stream, resize
	# it, and convert it to grayscale
	# channels)
	frame = vs.read()
	frame_count = frame_count+1
	frame = imutils.resize(frame, width=450)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the grayscale frame
	rects = detector.detectMultiScale(gray, scaleFactor=1.1, 
		minNeighbors=5, minSize=(30, 30),
		flags=cv2.CASCADE_SCALE_IMAGE)

	# loop over the face detections
	for (x, y, w, h) in rects:
		# construct a dlib rectangle object from the Haar cascade
		# bounding box
		rect = dlib.rectangle(int(x), int(y), int(x + w),
			int(y + h))

		# determine the facial landmarks for the face region, then
		# convert the facial landmark (x, y)-coordinates to a NumPy
		# array
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)

		# extract the left and right eye coordinates, then use the
		# coordinates to compute the eye aspect ratio for both eyes
		leftEye = shape[lStart:lEnd]
		rightEye = shape[rStart:rEnd]
		leftEAR = eye_aspect_ratio(leftEye)
		rightEAR = eye_aspect_ratio(rightEye)

		# average the eye aspect ratio together for both eyes
		ear = (leftEAR + rightEAR) / 2.0

		# compute the convex hull for the left and right eye, then
		# visualize each of the eyes
		leftEyeHull = cv2.convexHull(leftEye)
		rightEyeHull = cv2.convexHull(rightEye)
		lefteye_x = np.average(leftEye,axis=0)[0]
		lefteye_y = np.average(leftEye,axis=0)[1]
		righteye_x = np.average(rightEye,axis=0)[0]
		righteye_y = np.average(rightEye,axis=0)[1]

		eyes_y = (lefteye_y+righteye_y)/2

		head_positions.append(eyes_y)
		if len(head_positions)>10:
			head_positions.pop(0)

		if eyes_y>np.average(head_positions)-5:
			if counter_head<40 and head_down_flag:
				print("Warning HEAD")
				send_str(ser, "CMD_2\n")
				send_str(ser, '1')
			head_down_flag = False
			counter_head =0
		if eyes_y<np.average(head_positions)-10:
			counter_head=counter_head+1
			head_down_flag=True

		cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
		cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

		# check to see if the eye aspect ratio is below the blink
		# threshold, and if so, increment the blink frame counter
		if ear < EYE_AR_THRESH:
			COUNTER += 1

		# otherwise, the eye aspect ratio is not below the blink
		# threshold
		else:
			# if the eyes were closed for a sufficient number of
			# then increment the total number of blinks
			if COUNTER >= EYE_AR_CONSEC_FRAMES:
				blink_duration = COUNTER
				blink_durations_av = np.mean(blink_durations)
				blink_durations_std = np.std(blink_durations)
				blink_durations.append(blink_duration)
				if len(blink_durations) > 10:
					blink_durations.pop(0)
				if blink_durations_av>blink_duration-blink_durations_std:
					print("Warning Blink duration")
					send_str(ser, "CMD_2\n")
					send_str(ser, '2')
				TOTAL += 1

			# reset the eye frame counter
			COUNTER = 0

		if frame_count>100:
			blink_rate = TOTAL/frame_count
			blink_rates_std = np.std(blink_rates)
			blink_rates_av = np.mean(blink_rates)
			print("Current blink rate: ",blink_rate)
			print("Average blink rate :", blink_rates_av)
			print("STD: ", blink_rates_std)
			print("average + STD=", blink_rates_av+blink_rates_std)
			TOTAL=0
			frame_count=0
			if blink_rate > blink_rates_av+blink_rates_std:
				print("Warning Blink rate")
				send_str(ser, "CMD_2")
				send_str(ser, '3\n')
			blink_rates.append(blink_rate)
			if len(blink_rates) >10:
				blink_rates.pop(0)

		# draw the total number of blinks on the frame along with
		# the computed eye aspect ratio for the frame
		cv2.putText(frame, "Blinks: {}".format(TOTAL), (10, 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
		cv2.putText(frame, "EAR: {:.2f}".format(ear), (300, 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
		cv2.putText(frame, "Blink rate: {:.3f}".format(blink_rate), (10, 50),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
		cv2.putText(frame, "Avg. Blink rate: {:.3f}".format(np.average(blink_rates)), (10, 70),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
		cv2.putText(frame, "Blink duration: {:.3f}".format(blink_duration), (10, 290),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
		cv2.putText(frame, "Avg. Blink duration: {:.3f}".format(np.average(blink_durations)), (10, 310),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
		#print(ear)
	send_counter+=1
	print(send_counter)
	if send_counter>40:
		print("Sending parameters!")
		send_str(ser,"CMD_1")
		send_str(ser,"{:.2f}&{:.2f}&{:.2f}&{:.2f}&{:.2f}".format(ear,blink_rate,blink_rates_av,blink_duration,blink_durations_av))
		send_counter=0
	# show the frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()