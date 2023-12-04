import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
from std_msgs.msg import Int64 ,  String , Float32MultiArray
from dolasim_msgs.msg import IcVeriDolasim 


class WallFollowing(Node):
    def __init__(self):
        super().__init__('WallFollowing')
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.publisher1 = self.create_publisher(IcVeriDolasim,'/lidarlist',10)

        self.pub1 = self.create_publisher(LaserScan, "/sag", 10)
        self.pub2 = self.create_publisher(LaserScan, "/sagon", 10)
        self.pub3 = self.create_publisher(LaserScan, "/solon", 10)
        self.pub5 = self.create_publisher(LaserScan, "/sol", 10)
        self.pub4 = self.create_publisher(LaserScan, "/icveri",10)
        self.pub7 = self.create_publisher(LaserScan, "/lazer5", 10)
        self.pub8 = self.create_publisher(Int64,"/ldr7",10) 
        self.subscriber = self.create_subscription(LaserScan,'/scan',self.lıdar,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber
        self.timer_period = 0.2
        self.scan=[]
        self.out = IcVeriDolasim()
        self.ldr_sag= 0
        self.ldr_sol= 0
        self.ldr_on= 0

    def lıdar(self,msg):
        self.scan_data =msg
        scan1 = LaserScan() 
        scan2 = LaserScan()
        scan3 = LaserScan()
        scan4 = LaserScan()
        scan5 = LaserScan()

        scan1.header = self.scan_data.header
        scan2.header = self.scan_data.header
        scan3.header = self.scan_data.header
        scan4.header = self.scan_data.header
        scan5.header = self.scan_data.header
        
        scan1.angle_min = self.scan_data.angle_min
        scan2.angle_min = self.scan_data.angle_min
        scan3.angle_min = self.scan_data.angle_min
        scan4.angle_min = self.scan_data.angle_min
        scan5.angle_min = self.scan_data.angle_min

        scan1.angle_max = self.scan_data.angle_max   
        scan2.angle_max = self.scan_data.angle_max
        scan3.angle_max = self.scan_data.angle_max
        scan4.angle_max = self.scan_data.angle_max
        scan5.angle_max = self.scan_data.angle_max

        scan1.angle_increment = self.scan_data.angle_increment
        scan2.angle_increment = self.scan_data.angle_increment
        scan3.angle_increment = self.scan_data.angle_increment
        scan4.angle_increment = self.scan_data.angle_increment
        scan5.angle_increment = self.scan_data.angle_increment


        scan1.time_increment = self.scan_data.time_increment
        scan2.time_increment = self.scan_data.time_increment
        scan3.time_increment = self.scan_data.time_increment
        scan4.time_increment = self.scan_data.time_increment
        scan5.time_increment = self.scan_data.time_increment


        scan1.scan_time = self.scan_data.scan_time
        scan2.scan_time = self.scan_data.scan_time
        scan3.scan_time = self.scan_data.scan_time
        scan4.scan_time = self.scan_data.scan_time
        scan5.scan_time = self.scan_data.scan_time
        
        scan1.range_min = self.scan_data.range_min
        scan2.range_min = self.scan_data.range_min
        scan3.range_min = self.scan_data.range_min
        scan4.range_min = self.scan_data.range_min
        scan5.range_min = self.scan_data.range_min


        scan1.range_max = self.scan_data.range_max
        scan2.range_max = self.scan_data.range_max
        scan3.range_max = self.scan_data.range_max
        scan4.range_max = self.scan_data.range_max
        scan5.range_max = self.scan_data.range_max

        scan1.ranges[780:940] = self.scan_data.ranges[780:940] #on
        scan2.ranges[370:530] = self.scan_data.ranges[370:530] #sag
        scan3.ranges[1190:1350] = self.scan_data.ranges[1190:1350] # sol
        scan4.ranges[1700:1800] = self.scan_data.ranges[1700:2100]
        scan5.ranges[0:-1600] = self.scan_data.ranges[0:-1600]


        blg1=msg.ranges[780:820]
        blg2 = msg.ranges[820:860]
        blg3 = msg.ranges[860:900]
        blg4 = msg.ranges[900:940]
   


        self.blg11=msg.ranges[370:410]
        blg12=msg.ranges[410:450]
        blg13=msg.ranges[450:490]
        blg14=msg.ranges[490:530]

        

        blg16 =msg.ranges[1190:1230]
        blg17=msg.ranges[1230:1270]
        blg18=msg.ranges[1270:1310]
        blg19=msg.ranges[1310:1350]


        b1 =np.array(blg1)
        b2 = np.array(blg2)
        b3 = np.array(blg3)
        b4 = np.array(blg4)

        b11 =np.array(self.blg11)
        b12 =np.array(blg12)
        b13 =np.array(blg13)
        b14 =np.array(blg14)
 
        b16=np.array(blg16)
        b17 = np.array(blg17)
        b18=np.array(blg18)
        b19=np.array(blg19)

        data=[]
        data1=[]
        data2=[]
        self.data4 =[] 
        data5=[]
        data5.append(blg1)
        data5.append(blg2)
        data5.append(blg3)
        data5.append(blg4)

        data.append(b1)
        data.append(b2)
        data.append(b3)
        data.append(b4)
 
        self.scan=b11
      
        data1.append(b11)
        data1.append(b12)
        data1.append(b13)
        data1.append(b14)


        data2.append(b16)
        data2.append(b17)
        data2.append(b18)
        data2.append(b19)
        



        for j in range (len(data1)):
            self.veri = set(data1[j])
            
            if j <=2:
                d1 = set(data1[j])
                d2 = set(data1[j+1])
                if 0.0 in d1: 
                    d1.remove(0.0)
                if 0.0 in d2:  
                    d2.remove(0.0)
                try:
         
                    if min(d1)<2.0 and min(d2)<2.0:
                       # print(str(j) + " - " +str(j+1) +". Bölümün  sağında cisim var ")
                        self.ldr_sag=1.0
                    else:
                        print("engel yok ")
                        self.ldr_sag=0.0
                       
                except:
                    print("Dizi boş")    
  
            print("Sag Taraf : "+ str(self.ldr_sag))
        for k in range (len(data2)):
            if k <=2:
                d1 = set(data2[k])
                d2 = set(data2[k+1])
                if 0.0 in d1: 
                    d1.remove(0.0)
                if 0.0 in d2:  
                    d2.remove(0.0)
                try:
                 
                    if min(d1)<2.0 and min(d2)<2.0:
                       print(str(k) + " - " +str(k+1) +". Bölümün  solunda cisim var ")
                       self.ldr_sol=1.0
                    # print("sol"+self.ldr_sol)
                    else:
                        print("engel yok ")
                        self.ldr_sol=0.0
                except:
                    print("Dizi boş")    

            print("Sol Taraf : "+str(self.ldr_sol))
        bakı = Int64()

        for i in range(len(data5)):
          
            if i<=2:
                s1 = set(data5[i])
                s2 = set(data5[i+1])
                if 0.0 in s1: 
                    s1.remove(0.0)
                if 0.0 in s2:  
                    s2.remove(0.0)
                try:
                   
                    if min(s1)<2.0 and min(s2)<2.0:
                        print(str(i) + " - " +str(i+1) +". Bölümün ön tarafında cisim var ")
                        self.ldr_on=1.0
                     
                    else:
                        print("Engel yok")  
                        self.ldr_on=0.0                   
                except:
                    print("Dizi boş")

            print("On Taraf : "+str(self.ldr_on))  
                 

            print("Lidarr: "+str(bakı.data))    
       
        #self.out.ldr_sol=self.ldr_sol
        #self.out.ldr_sag=self.ldr_sag
        #self.out.ldr_sag=self.ldr_on
        

        self.pub1.publish(scan1) 
        self.pub2.publish(scan2)
        self.pub3.publish(scan3) 
        self.pub5.publish(scan4)
        self.pub7.publish(scan5)
        self.pub8.publish(bakı)
        self.publisher1.publish(self.out)


def main(args=None):
    rclpy.init(args=args)
    wall_following = WallFollowing()
    try:
        rclpy.spin(wall_following)
    except KeyboardInterrupt:
        pass
    wall_following.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
