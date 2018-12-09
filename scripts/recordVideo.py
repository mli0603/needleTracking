import cv2
import numpy as np

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280*2)


fourcc = cv2.VideoWriter_fourcc(*'XVID')
outL = cv2.VideoWriter('videoLeft.avi',fourcc, 120.0, (1280,960))
outR = cv2.VideoWriter('videoRight.avi',fourcc, 120.0, (1280,960))


cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
while(True):
	ret, frame = cap.read()

	# Display the resulting frame
	cv2.imshow('frame',frame)

	# write the flipped frame
	outL.write(frame[:, :int(frame.shape[1] / 2), :])
	outR.write(frame[:, int(frame.shape[1] / 2):, :])

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

#when everything done, release the capture
cap.release()
cv2.destroyAllWindows()
