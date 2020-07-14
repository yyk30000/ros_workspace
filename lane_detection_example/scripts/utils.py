import numpy as np
import cv2
import math
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

def traslationMtx(x,y,z):
    M =np.array([[1,0,0,x],
    [0,1,0,y],
    [0,0,1,z],
    [0,0,0,1],
    ])
    return M


def rotationMtx(yaw,pitch,roll):
    R_x =np.array(
    [[1,0,0,0],
    [0,math.cos(roll),-math.sin(roll),0],
    [0,math.sin(roll),math.cos(roll),0],
    [0,0,0,1],
    ])
    R_y=np.array([[math.cos(pitch),0,math.sin(pitch),0],
                [0,1,0,0],
                [-math.sin(pitch),0,math.cos(pitch),0],
                [0,0,0,1],
                ])
    R_z=np.array([[math.cos(yaw),-math.sin(yaw),0,0],
                [math.cos(yaw),math.cos(yaw),0,0],
                [0,0,1,0],
                [0,0,0,1],
                ])
    R=np.matmul(R_x,np.matmul(R_y,R_z))
    return R

def project2img_mtx(params_cam):
    fc_x =params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y =params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    cx =params_cam["WIDTH"]/2
    cy =params_cam["HEIGHT"]/2

    R_f=np.array([[fc_x ,0,cx],
                [0,fc_y,cy]])
    return R_f

class BEVTransform:
    def __init__(self,params_cam,xb=1.0,zb=1.0):
        self.xb=xb
        self.zb=zb

        self.theta =np.deg2rad(params_cam["PITCH"])
        self.width =params_cam["WIDTH"]
        self.height =params_cam["HEIGHT"]

        if params_cam["ENGINE"] =="UNITY":
            self.alpha_r =np.deg2rad(params_cam["FOV"]/2)
            self.fc_y=params_cam["HEIGHT"]/(2*np.tan(np.deg2rad([params_cam["FOV"]/2])))
            self.alpha_c =np.arctan2(params_cam["WIDTH"]/2,self.fc_y)

            self.fc_x =self.fc_y

        else:
            self.alpha_c =np.deg2rad(params_cam["FOV"]/2)
            self.fc_x=params_cam["HEIGHT"]/(2*np.tan(np.deg2rad([params_cam["FOV"]/2])))
            self.alpha_r =np.arctan2(params_cam["WIDTH"]/2,self.fc_x)

            self.fc_y =self.fc_x

        self.h = params_cam["Z"]

        self.n =float(params_cam["WIDTH"])
        self.m =float(params_cam["HEIGHT"])

        self.RT_b2g =np.matmul(np.matmul(traslationMtx(xb,0,zb),rotationMtx(np.deg2rad(-90),0,0)),
                                rotationMtx(0,0,np.deg2rad(180)))
        self.build_tf(params_cam)                        