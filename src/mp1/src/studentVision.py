import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology
import matplotlib.pyplot as plt



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        #830
        self.sub_image = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1)

        # Uncomment this line for lane detection of videos in rosbag
        self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to uint8, then apply threshold to get binary image

        ## TODO

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), cv2.BORDER_DEFAULT)
       
        sobelx = cv2.Sobel(blur, ddepth=cv2.CV_8U, dx=1, dy=0, ksize=3)
        sobely = cv2.Sobel(blur, ddepth=cv2.CV_8U, dx=0, dy=1, ksize=3)

        res = cv2.addWeighted(sobelx, 0.5, sobely, 0.5, 0.0)
        res = np.array(res, dtype=np.uint8)
        
        res[ thresh_max < res] = 0

        res[ res < thresh_min] = 0
        res[ (thresh_max >= res) & (res >= thresh_min)] = 1
        
        
        return res


    def color_thresh(self, img, thresh=(100, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO

        img1 = img


       
        # cv2.imshow("test", img)
        # cv2.waitKey()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        lower_green = np.array([40, 40,40], dtype = np.uint8)
        upper_green = np.array([70, 255,255], dtype = np.uint8)

        green_mask = cv2.inRange(img, lower_green, upper_green )

        img = cv2.cvtColor(cv2.bitwise_and(img,img, mask=green_mask),cv2.COLOR_HLS2BGR)
        img[img> 0] = 255
        img = (255 - img)

        img = cv2.bitwise_and(img1,img)


        # # h,l,s = cv2.split(cv2.cvtColor(img,cv2.COLOR_BGR2HLS))

        # # h[(h>=15) & (h<45)] = 
        # # img = cv2.merge((h,l,s))

        #gazebo
        lower_white = np.array([0, 180,0], dtype = np.uint8)
        upper_white = np.array([180, 255,255], dtype = np.uint8)

        lower_yellow = np.array([20, 0,90], dtype = np.uint8)
        upper_yellow = np.array([30, 255,255], dtype = np.uint8)


        #others
        # lower_white = np.array([0, 190,0], dtype = np.uint8)
        # upper_white = np.array([180, 255,90], dtype = np.uint8)

        # lower_yellow = np.array([20, 0,90], dtype = np.uint8)
        # upper_yellow = np.array([30, 255,255], dtype = np.uint8)


        #831
        # lower_white = np.array([0, 120,0], dtype = np.uint8)
        # upper_white = np.array([180, 255,130], dtype = np.uint8)

        # lower_yellow = np.array([15, 0,80], dtype = np.uint8)
        # upper_yellow = np.array([45, 255,255], dtype = np.uint8)

        
        

        # img = cv2.cvtColor(cv2.bitwise_and(img,img, mask=white_mask),cv2.COLOR_HLS2BGR)

        image_hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        white_mask = cv2.inRange(image_hls, lower_white, upper_white)
        
        yellow_mask = cv2.inRange(image_hls, lower_yellow, upper_yellow)
       
        final_mask = cv2.bitwise_or(white_mask, yellow_mask)

        filtered_image_hls = cv2.bitwise_and(image_hls, image_hls, mask=final_mask)


        filtered_image_bgr = cv2.cvtColor(filtered_image_hls, cv2.COLOR_HLS2BGR)
        # cv2.imshow("SS", filtered_image_bgr)
        # cv2.waitKey()

        filtered_image_bgr = cv2.cvtColor(filtered_image_hls, cv2.COLOR_BGR2GRAY)

        filtered_image_bgr[filtered_image_bgr>0] = 255 #max is 255 instead of 1

        # plt.imshow(filtered_image_bgr)
        # plt.show()
    

        ####

        return filtered_image_bgr


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ColorOutput = self.color_thresh(img)/255
        SobelOutput = self.gradient_thresh(img)/255


        ####

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
             
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)

        # plt.imshow(ColorOutput)
        # plt.show
        return ColorOutput


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        # plt.imshow(img)
        # plt.show()
        
        maxHeight = len(img)
        maxWidth = len(img[0])
        #gazebo
        input_pts = np.float32([[160,290], [0, 370], [616, 370], [450,290]])
        #011
        #input_pts = np.float32([[546,215], [300, 375], [843, 375], [711,215]])
        #056
        #input_pts = np.float32([[504,236], [269, 375], [830, 375], [701,236]])
        #0830
        # input_pts = np.float32([[530,386], [177, 688], [1010, 688], [720,386]])

        output_pts = np.float32([[0, 0],
                        [0, maxHeight - 1],
                        [maxWidth - 1, maxHeight - 1],
                        [maxWidth - 1, 0]])
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        Minv = np.linalg.inv(M)
        #cv2.imshow("plz" , img.astype('uint8')*255)
        
        warped_img = cv2.warpPerspective(img.astype('uint8')*255,M,(maxWidth, maxHeight),flags=cv2.INTER_LINEAR)/255
        #cv2.imshow("plz" , warped_img/255)
        #cv2.waitKey()
        ####

        return warped_img, M, Minv


    def detection(self, img):
        binary_img = self.combinedBinaryImage(img)

        cv2.imwrite("testing.png", img)

        img_birdeye, M, Minv = self.perspective_transform(binary_img)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img
        
        


if __name__ == '__main__':
    # init args
    # source1 = cv2.imread('src/mp1/src/test.png', cv2.IMREAD_COLOR)

    # lanenet_detector.perspective_transform(lanenet_detector, source1)    
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
