import numpy as np
import cv2

def warp_image(img,source_prop):

    image_size = (img.shape[1],img.shape[0])
    x = img.shape[1]
    y = img.shape[0]

    destination_points = np.float32([
        [0,y],
        [0,0],
        [x,0],
        [x,y]
    ])

    source_points =source_prop * np.float32([
        [x,y],
        [x,y],
        [x,y],
        [x,y]
    ])

    perspective_transform =cv2.getPerspectiveTransform(source_points,destination_points)

    warped_img =cv2.warpPerspective(img,perspective_transform, image_size, flags=cv2.INTER_LINEAR)
    
    return warped_img