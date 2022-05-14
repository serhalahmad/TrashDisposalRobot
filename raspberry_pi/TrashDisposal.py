"""Main script to run the object detection routine."""
import argparse
import sys
import time
from multiprocessing import Process, Queue
import FuzzyController as fz
import RoboticArm as TrashArm

import RPi.GPIO as GPIO

import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
import utils

#values for paper 30,5,303

#Reference values for the cardboard box
known_distance = 45
known_width = 20
ref_image_width = 552

fonts = cv2.FONT_HERSHEY_COMPLEX

counter = 6

GREEN = (0, 255, 0)

#Initialize the rover's motors
GPIO.setmode(GPIO.BOARD)

GPIO.setwarnings(False)
ENL,CCWL,CWL = 12,7,11
ENR,CCWR,CWR = 32,13,15
GPIO.setup(ENL,GPIO.OUT)
GPIO.setup(CCWL,GPIO.OUT)
GPIO.setup(CWL,GPIO.OUT)
GPIO.setup(ENR,GPIO.OUT)
GPIO.setup(CCWR,GPIO.OUT)
GPIO.setup(CWR,GPIO.OUT)

centerX = 0

# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):

	# finding the focal length
	focal_length = (width_in_rf_image * measured_distance) / real_width
	return focal_length

def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):

	distance = (real_face_width * Focal_Length)/face_width_in_frame

	# return the distance
	return distance

def run(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
  """
  # Variables for distance measurement
  face_width_in_frame = 640
  xleft, xright, distance = 0,0,0
  true_focal_length = 686
  
  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  # Initialize the object detection model
  options = ObjectDetectorOptions(
      num_threads=num_threads,
      score_threshold=0.4,
      max_results=3,
      enable_edgetpu=enable_edgetpu)
  detector = ObjectDetector(model_path=model, options=options)

  # Continuously capture images from the camera and run inference
  while cap.isOpened():
    
    
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam settings.'
      )

    counter += 1
    image = cv2.flip(image,0)

    # Run object detection estimation using the model.
    detections = detector.detect(image)

    #Determine pixel width of object in frame
    for detection in detections:    
        xleft = detection.bounding_box.left
        xright = detection.bounding_box.right

    face_width_in_frame = xright - xleft
    
    #Find the x coordinate of center of object
    xcenter = xleft + (face_width_in_frame/2)
    
    print("From inference ")
    print(xcenter)
        
    #Find focal length (Not Used In This Case Since We Defined a Set Focal Length)
    #focal_length_found = Focal_Length_Finder(known_distance, known_width, ref_image_width)

    #Find distance
    if face_width_in_frame > 1:
        #Find the distance between the camera and object
        distance =  Distance_finder(true_focal_length, known_width, face_width_in_frame)

        #Show the distance
        cv2.putText(image, f"Distance: {round(distance,2)} CM", (30, 35),
		    fonts, 0.6, GREEN, 2)
    
    if counter == 8:
        q.put(xcenter)
        d.put(distance)
        counter = 0
   
    # Draw keypoints and edges on input image
    image = utils.visualize(image, detections)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    #Show the distance
    cv2.putText(image, f"Distance: {round(distance,2)} CM", (30, 35),
		fonts, 0.6, GREEN, 2)
    
    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
      break
    cv2.imshow('object_detector', image)

  cap.release()
  cv2.destroyAllWindows()

def move():
    #Start up the rover's motors in idle
    pwm1 = GPIO.PWM(ENL,100)
    pwm2 = GPIO.PWM(ENR,100)
    pwm1.start(0)
    pwm2.start(0)
    
    while True:
   
        #Get the x coordinate center value of the box from the object detection process
        centerX = q.get()

        print("From move")
        print(centerX)
        
        #distancetoCenter is the distance from the center of the frame to the center of the object
        #centerX is the center coordinate with respect to the X-Y coordinate system of the frame
        distancetoCenter = centerX - 320
        distanceToRover = round(d.get(),1)
        #Rotate left
        if distancetoCenter > 30:
            GPIO.output(CCWL,GPIO.LOW) 
            GPIO.output(CWL,GPIO.HIGH)
            GPIO.output(CCWR,GPIO.HIGH)
            GPIO.output(CWR,GPIO.LOW)
            dutyCycle= fz.rotation_controller(distancetoCenter)
            
            pwm1.ChangeDutyCycle(dutyCycle)
            pwm2.ChangeDutyCycle(dutyCycle)
            time.sleep(0.5)
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
            time.sleep(4)

        #Rotate right         
        if distancetoCenter < -30:
            GPIO.output(CCWL,GPIO.HIGH) 
            GPIO.output(CWL,GPIO.LOW)
            GPIO.output(CCWR,GPIO.LOW)
            GPIO.output(CWR,GPIO.HIGH)
            dutyCycle = fz.rotation_controller(distancetoCenter)
            
            pwm1.ChangeDutyCycle(dutyCycle)
            pwm2.ChangeDutyCycle(dutyCycle)
            time.sleep(0.5)
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
            time.sleep(4)
        
        #Go to box once the rover is centered with respect to the box
        if (distancetoCenter < 35.0) & (distancetoCenter > -35):
            GPIO.output(CCWL,GPIO.HIGH) 
            GPIO.output(CWL,GPIO.LOW)
            GPIO.output(CCWR,GPIO.LOW)
            GPIO.output(CWR,GPIO.HIGH)
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
            if distanceToRover > 25:
                dutyCycle = fz.forward_controller(distanceToRover)
                GPIO.output(CCWL,GPIO.LOW) 
                GPIO.output(CWL,GPIO.HIGH)
                GPIO.output(CCWR,GPIO.LOW)
                GPIO.output(CWR,GPIO.HIGH)
                pwm1.ChangeDutyCycle(dutyCycle)
                pwm2.ChangeDutyCycle(dutyCycle)
                time.sleep(0.8)
                pwm1.ChangeDutyCycle(0)
                pwm2.ChangeDutyCycle(0)
                time.sleep(4)
            else: #Once the rover is close enough to the box -> disposes trash
                GPIO.output(CCWL,GPIO.LOW) 
                GPIO.output(CWL,GPIO.HIGH)
                GPIO.output(CCWR,GPIO.LOW)
                GPIO.output(CWR,GPIO.HIGH)
                pwm1.ChangeDutyCycle(100)
                pwm2.ChangeDutyCycle(100)
                time.sleep(0.8)
                pwm1.ChangeDutyCycle(0)
                pwm2.ChangeDutyCycle(0)
                time.sleep(4)
                print("Disposing Trash")
                TrashArm.disposeTrash()
                print("Done")
                break

def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='cbox.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=False)
  args = parser.parse_args()
  
  
  TrashArm.openClipper()
  time.sleep(5)
  TrashArm.closeClipper()
  
  
  movement = Process(target = move)
  movement.start()  
  objectDetection = Process(target = run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
                                int(args.numThreads), bool(args.enableEdgeTPU)))
  objectDetection.start()
  
#Standard run for object detection only
  #run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
    #   int(args.numThreads), bool(args.enableEdgeTPU))


if __name__ == '__main__':
    d = Queue()
    q = Queue()
    main()
  
  
