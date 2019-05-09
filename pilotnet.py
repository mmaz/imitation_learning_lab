import cv2 as cv

# images are 320 (width) x 240 (height) from the logitech webcam

# pilotnet parameters for network dimension:

IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 120, 280, 3
INPUT_SHAPE = (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS)


# magnitude of your estimated steering angle correction for the left/right cameras

OFFSET_STEERING_ANGLE = 0.1 # NOTE: use two values if the left/right camera angles are not symmetric

# some convenience functions for preprocessing frames:

# original Nvidia paper uses these dimensions
def pilotnet_crop(image):
    """assumes 320x240 input, resizes to 200x66"""
    #      (original - target)
    # rows:    (240 - 66) / 2 == 87
    # columns: (320 - 200) /2 == 60 
    return image[87:-87, 60:-60] 

# You might want to grab more of the image area and shrink it 
# down (instead of just cropping the center of the image out), e.g.,
# cv.resize(image,(0,0), fx=0.4, fy=0.4, interpolation=cv.INTER_AREA )
def shrink(image):
    return cv.resize(image, (200,66), cv.INTER_AREA)

# you can try using a larger crop:
def pilotnet_crop_large(image):
    """assumes 320x240 input, resizes to 280x120"""
    #       (original - target)
    # rows:      (240 - 120) / 2 == 60
    # columns:   (320 - 280) /2 == 35
    return image[60:-60, 20:-20] 


def preprocess(image, use_full_frame=False):
    if use_full_frame:
        return shrink(image)
    return pilotnet_crop_large(image)
