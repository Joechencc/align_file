from sensor_msgs.msg import CameraInfo, Image
import message_filters
import rospy
from cv_bridge import CvBridge
import cv2
from os import path

count = 0
init_sec = 0
init_nsec = 0
sec = 0
nsec = 0

def detect(image, depth, image_info, depth_info):
    global count
    global init_sec, init_nsec
    #cv_image = CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8')
    cv_image = CvBridge().imgmsg_to_cv2(depth, desired_encoding="passthrough")

    if (init_sec == 0):
        init_sec = image_info.header.stamp.secs
        init_nsec = image_info.header.stamp.nsecs
    sec = image_info.header.stamp.secs
    nsec = image_info.header.stamp.nsecs

    time = (sec - init_sec) + 1e-9 * (nsec - init_nsec)

    #file_path = "/home/chen1804/Desktop/align_file/save_file_1"
    file_path = "/home/chen1804/Desktop/align_file/save_file_1_depth"
    #file_name = "image_"+str(count)+ "_"+ str(round(time,2)) + ".jpg"
    file_name = "depth_"+str(count)+ "_"+ str(round(time,2)) + ".jpg"
    cv2.imwrite(path.join(file_path, file_name), cv_image)
    count = count + 1
    cv2.waitKey(30)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    while not rospy.is_shutdown():
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size = 1, buff_size=2**24)
        image_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size = 1, buff_size=2**24)
        depth_info = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size = 1, buff_size=2**24)
        images_info = message_filters.Subscriber("/camera/color/camera_info", CameraInfo, queue_size = 1, buff_size=2**24)
        ats = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub, images_info, depth_info], 10,1 )
        ats.registerCallback(detect)
        rospy.spin()
