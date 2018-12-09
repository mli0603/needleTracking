import cv2

def onBrightness(val):
	b = float(val)/100
	cap.set(cv2.CAP_PROP_BRIGHTNESS,b)

def onContrast(val):
	val = float(val)/100
	cap.set(cv2.CAP_PROP_CONTRAST,val)

def onSaturation(val):
	val = float(val)/100
	cap.set(cv2.CAP_PROP_SATURATION,val)

def onGain(val):
	val = float(val)/100
	cap.set(cv2.CAP_PROP_GAIN,val)


cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280*2)

winName = 'trackbars'
cv2.namedWindow(winName)

# create trackbars for cam parameter change
cv2.createTrackbar( "Brightness",winName, 0, 100, onBrightness );
cv2.createTrackbar( "Contrast",winName, 0, 100,onContrast );
cv2.createTrackbar( "Saturation",winName, 0, 100,onSaturation);
cv2.createTrackbar( "Gain",winName, 0, 100,onGain);


cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
while(True):
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
    	cv2.imwrite("imgLData15.jpg",frame[:, :int(frame.shape[1] / 2), :])
    	cv2.imwrite("imgRData15.jpg",frame[:, int(frame.shape[1] / 2):, :])
        break#Display resulting frame

#when everything done, release the capture
cap.release()
cv2.destroyAllWindows()