#!/home/lhy/python38/bin/python3

from audioop import ratecv
import sys
import rospy
from threading import Thread
from queue import Queue
from tactile_paving.msg import signalLamp


FILE_PATH = sys.path[0]
sys.path.append(FILE_PATH+"/PaddleDetection")
from PaddleDetection.paddle_dog import ros_main


def traffic_lights_decision_publisher():
    rospy.init_node("traffic_lights_decision",anonymous=True)
    turtle_vel_pub = rospy.Publisher("signalLamp", signalLamp,queue_size=10)
    process_ = Thread(target=ros_main,
            args=(udp_cam_id,udp_ip_last_segment,model_dir,queue,FILE_PATH+"/PaddleDetection/output"))
    process_.start()
    while not rospy.is_shutdown():
        paddle_data = queue.get()
        if paddle_data[1]<0.4:
            continue
        signalLamp_msg = signalLamp()
        signalLamp_msg.lamp_class = str(paddle_data[0])
        signalLamp_msg.xmin = int(paddle_data[2])
        signalLamp_msg.ymin = int(paddle_data[3])
        signalLamp_msg.xmax = int(paddle_data[4])
        signalLamp_msg.ymax = int(paddle_data[5])
        signalLamp_msg.fps = int(paddle_data[6])
        turtle_vel_pub.publish(signalLamp_msg)
    process_.join()




if __name__ == "__main__":
    udp_cam_id   = int(rospy.get_param("/cam_up","1"))
    udp_ip_last_segment = rospy.get_param("/IpLastSegment","15")
    model_dir = rospy.get_param("/paddle_model_path",FILE_PATH+"/PaddleDetection/output_inference/picodet_m_320_coco_lcnet/")
    queue = Queue()
    try:
        traffic_lights_decision_publisher()
    except rospy.ROSInterruptException:
        pass

