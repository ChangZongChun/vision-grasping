#!/usr/bin/python
import freenect
import numpy as np
import cv2

def get_video():
    array,_ = freenect.sync_get_video(0,freenect.VIDEO_IR_10BIT)
    return array
def pretty_depth(depth):

    np.clip(depth, 0, 2**10-1, depth)
    depth >>=2
    depth=depth.astype(np.uint8)
    return depth
if __name__ == "__main__":

    img_counter = 0

    while 1:
        #get a frame from RGB camera
        frame = get_video()
        #display IR image
        frame = pretty_depth(frame)

        cv2.imshow('IR image',frame)

        # quit program when 'esc' key is pressed
        k = cv2.waitKey(100) & 0xFF
        if k == 27:
            break

        if k == ord('s'):
            img_name = "IR_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1
    
    cv2.destroyAllWindows()
