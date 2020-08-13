class cruiseControl:
    def __init__(self,object_vel_gain,object_dis_gain):
        self.object=[False,0]
        self.stop_lane=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,ref_path,global_vaild_object,local_vaild_object):
            self.object=[False,0]
            self.stop_lane=[False,0]

            if len(global_vaild_object) >0 :
                min_rel_distance=float('inf')

                for i range(len(global_vaild_object)):

                    for path in ref_path.poses :

                        if global_vaild_object[i][0]==2:

                            dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                            if dis < 2:
                                rel_distance = sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                if rel_distance < min_rel_distance:
                                    min_rel_distance = rel_distance
                                    self.stop_lane=[False,0]
                                    self.object=[True,i]

                                break    


                            if global_vaild_object[i][0]==1  :

                                dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][2],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                                if dis<1.5 :
                                    rel_distance =sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                    if rel_distance < min_rel_distance:
                                        min_rel_distance = rel_distance
                                        self.object=[Fasle,0]
                                        self.stop_lane[True,i]

    def acc(self,local_vaild_object,ego_vel,target_vel):
        out_vel=target_vel

        if self.object[0] == True :
            print("ACC ON")
            front_vehicle=[local_vaild_object[self.object[1]][1],local_vaild_object[self.object[1]][3]]            
            time_gap=1
            default_space=2
            dis_safe=ego_vel* time_gap + default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-4.4
            print(dis_rel,dis_rel)
            vel_rel=(front_vehicle[2]-ego_vel)*3.6 
            v_gain =self.object_dis_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain-x_errgain*(dis_safe-dis_rel)

            acc_based_vel=ego_vel*3.6+acceleration

            if acc_based_vel > target_vel:
                acc_based_vel =target_vel

            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel:
                out vel == acc_based_vel                           