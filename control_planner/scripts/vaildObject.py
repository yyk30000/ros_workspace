class vaildObject :

    def __init__(self):

        self.vaild_stoplane_position=[
                                     [58.26,1180.09],
                                     [85.56,1228.28]
                                     ]

    def get_object(self,obj_msg):
        self.all_object=ObjectInfo()
        self.all_object=obj_msg


    def calc_vaild_obj(self,ego_pose):
        global_object_info=[]
        loal_object_info=[]
        if self.all_object.num_of_objects > 0:

            tmp_theta = ego_pose[2]
            tmp_translation=[ego_pose[0],ego_pose[1]]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                            [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                            [0,0,1]])

            tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])  ],
                                [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])  ],
                                [0,0,1]])

            for num in range(self.all_object.num_of_objects):
                global_result=np.array([[self.all_object.pose_x[num]],[self.all_object.pose_y[num],[1]]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]>0 :
                    global_object_info.append([self.all_object_type[num],self.all_object.pose_x[num],self.all_object.pose_y[num],self.all_object.velocity[num]])                    
                    loal_object_info.append([self.all_object.object_type[num],local_result[0][0],local_result[0][0],local_result[1][0],self.all_object.velocity[num]])  


            for num in range(len(self.vaild_stoplane_position)):
                global_result=np.array([[self.vaild_stoplane_position[num][0]],[self.vaild_stoplane_position[num][1]],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]>0 :
                    global_object_info.append([1,self.all_object.pose_x[num],self.all_object.pose_y[num],0])
                    loal_object_info.append([1,local_result[0][0],local_result[1][0],0])


        return global_object_info,loal_object_info                               