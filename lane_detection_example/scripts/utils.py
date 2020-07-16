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
        self.x =0

        self.proj_mtx=project2img_mtx(params_cam)
        
        self.theta =np.deg2rad(params_cam["PITCH"])
        self.width =params_cam["WIDTH"]
        self.height =params_cam["HEIGHT"]

        if params_cam["ENGINE"] =="UNITY":
            self.alpha_r =np.deg2rad(params_cam["FOV"]/2)

            self.fc_y=params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_c =np.arctan2(params_cam["WIDTH"]/2,self.fc_y)

            self.fc_x =self.fc_y

        else:
            self.alpha_c =np.deg2rad(params_cam["FOV"]/2)

            self.fc_x=params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_r =np.arctan2(params_cam["WIDTH"]/2,self.fc_x)

            self.fc_y =self.fc_x

        self.h = params_cam["Z"]

        self.n =float(params_cam["WIDTH"])
        self.m =float(params_cam["HEIGHT"])

        self.RT_b2g =np.matmul(np.matmul(traslationMtx(xb,0,zb),rotationMtx(np.deg2rad(-90),0,0)),
                                rotationMtx(0,0,np.deg2rad(180)))
        self.build_tf(params_cam)


    def calc_Xv_Yu(self,U,V):

        Xv=self.h*(np.tan(self.theta)*(1-2*(V-1)/self.m-1)*np.tan(self.alpha_r)-1)/(-np.tan(self.theta)+(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r))

        Yu=(1-2*(U-1)/(self.n-1))*Xv*np.tan(self.alpha_c)

        return Xv,Yu

    def build_tf(self,params_cam):

        v =np.array([params_cam["HEIGHT"]*0.5,params_cam["HEIGHT"]]).astype(np.float32)
        u =np.array([0,params_cam["WIDTH"]]).astype(np.float32)

        U, V =np.meshgrid(u,v)

        Xv,Yu =self.calc_Xv_Yu(U, V)

        xyz_g =np.concatenate([Xv.reshape([1,-1])+params_cam["X"],
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))],axis=0)
        
        xyz_bird = np.matmul(np.linalg.inv(self.RT_b2g),xyz_g)

        xc, yc , zc =xyz_bird[0,:].reshape([1,-1]),xyz_bird[1,:].reshape([1,-1]),xyz_bird[2,:].reshape([1,-1])
        
        self.proj_mtx =project2img_mtx(params_cam)

        xn,yn =xc/zc,yc/zc

        xyi= np.matmul(self.proj_mtx,np.concatenate([xn,yn,np.ones_like(xn)],axis=0))

        xyi=xyi[0:2,:].T

        src_pts=np.concatenate([U.reshape([-1,1]),V.reshape([-1,1])],axis=1).astype(np.float32)
        dst_pts=xyi.astype(np.float32)

        self.perspective_tf =cv2.getPerspectiveTransform(src_pts,dst_pts)
    
    
    def warp_bev_img(self,img):
        img_warp =cv2.warpPerspective(img,self.perspective_tf,(self.width,self.height),flags=cv2.INTER_LINEAR)
        return img_warp


    def project_lane2img(self,x_pred,y_pred_l,y_pred_r):
        xyz_l_g =np.concatenate([x_pred.reshape([1,-1]),
                                y_pred_l.reshape([1,-1]),
                                np.zeros_like(y_pred_l.reshape([1,-1])),
                                np.ones_like(y_pred_l.reshape([1,-1]))
                                ],axis= 0)

        xyz_r_g =np.concatenate([x_pred.reshape([1,-1]),
                                y_pred_r.reshape([1,-1]),
                                np.zeros_like(y_pred_r.reshape([1,-1])),
                                np.ones_like(y_pred_r.reshape([1,-1]))
                                ],axis= 0)

        xyz_l_b =np.matmul(np.linalg.inv(self.RT_b2g),xyz_l_g)
        xyz_r_b =np.matmul(np.linalg.inv(self.RT_b2g),xyz_r_g)

        xyl =self.project_pts2img(xyz_l_b)
        xyr =self.project_pts2img(xyz_r_b)

        xyl =self.crop_pts(xyl)
        xyr =self.crop_pts(xyr)

        return xyl,xyr

       

    def project_pts2img(self,xyz_bird):
        xc ,yc,zc =xyz_bird[0,:].reshape([1,-1]),xyz_bird[1,:].reshape([1,-1]),xyz_bird[2,:].reshape([1,-1])
        xn,yn = xc/(zc+0.0001),yc/(zc+0.0001)

        xyi =np.matmul(self.proj_mtx,np.concatenate([xn,yn,np.ones_like(xn)],axis=0))
        xyi =xyi[0:2,:].T
        return xyi

    def crop_pts(self ,xyi):
        xyi = xyi[np.logical_and(xyi[:,0]>=0,xyi[:,0]<self.width),:]
        xyi = xyi[np.logical_and(xyi[:,0]>=0,xyi[:,0]<self.height),:]

        return xyi
    def recon_lane_pts(self,img):
        img[:int(0.5*self.height),:]=0

        if cv2.countNonZero(img) != 0:

            UV_mark=cv2.findNonZero(img).reshape([-1,2])

            U,V =UV_mark[:,0].reshape([-1,1]),UV_mark[:,1].reshape([-1,1])

            Xv,Yu =self.calc_Xv_Yu(U,V)

            xyz_g =np.concatenate([Xv.reshape([1,-1]) + self.x,
                                    Yu.reshape([1,-1]),
                                    np.zeros_like(Yu.reshape([1,-1])),
                                    np.ones_like(Yu.reshape([1,-1]))], axis= 0)


            xyz_g = xyz_g[:,xyz_g[0,:]>=0]

        else:
            xyz_g =np.zeros((4,10))

        return xyz_g










