import cv2 as cv
import numpy as np
import cameras_RACECAR as dev

def label_device(msg, img):
    height, width = img.shape[0], img.shape[1]
    x, y = int(width / 3.), int(height / 2.)

    font = cv.FONT_HERSHEY_SIMPLEX
    label = "{}".format(msg)
    cv.putText(img,label,(x,y), font, 1.0,(255,255,255),3,cv.LINE_AA)
    return img

def main():
    dev.Video.notify()

    WEBCAMS = "WEBCAMS"

    cv.namedWindow(WEBCAMS)
    cap_l = cv.VideoCapture(dev.Video.LEFT)
    cap_l.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cap_l.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    cap_c = cv.VideoCapture(dev.Video.CENTER)
    cap_c.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cap_c.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    cap_r = cv.VideoCapture(dev.Video.RIGHT)
    cap_r.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cap_r.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
            ret_l, frame_l = cap_l.read()
            ret_c, frame_c = cap_c.read()
            ret_r, frame_r = cap_r.read()
            frame_l = label_device("LEFT: {}".format(dev.Video.LEFT),     frame_l)
            frame_c = label_device("CENTER: {}".format(dev.Video.CENTER), frame_c)
            frame_r = label_device("RIGHT: {}".format(dev.Video.RIGHT),   frame_r)
            frames = np.hstack([frame_l, frame_c, frame_r])
            cv.imshow(WEBCAMS, frames)
            cv.waitKey(1)

    cap_l.release()
    cap_c.release()
    cap_r.release()
    cv.destroyAllWindows()



if __name__ == "__main__":
    main()



