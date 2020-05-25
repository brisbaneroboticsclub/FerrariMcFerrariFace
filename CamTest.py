import cv2

'''
http://brisbaneroboticsclub.id.au/ferrari-mcferrari-face/

To run code
$ python3 CamTest.py
'''

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    print(ret)
    #print(frame)

    print('To exit, click on frame and hit q.')

    # Display the resulting frame
    RowCt, ColCt, Dim = frame.shape
    frame = cv2.resize(frame, (int(0.3 * ColCt), int(0.3 * RowCt)), interpolation=cv2.INTER_CUBIC)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print('Exiting now.')
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()