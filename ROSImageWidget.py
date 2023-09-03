#!/usr/bin/python3

from kivy.clock import Clock
from kivy.properties import ListProperty
from kivy.graphics.texture import Texture
from kivy.uix.image import Image as KivyImage

from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image

class ROSImageWidget(KivyImage):
    '''
    Widget that allows for the visualization of an image topic into a kivy app easily
    '''
    __events__ = ('on_texture_update',)
    frame_size = ListProperty([None,None])

    def __init__(self, image_topic:str = "/camera/color/image_raw", frame_rate:int = 60, **kwargs):
        super().__init__(**kwargs)
        
        self.bridge = CvBridge()
        self.img_updater = Clock.schedule_interval(self.update_frames, 1.0/frame_rate)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.cv2_img_callback, queue_size=1)
        
        self.frame = None
        self._buffer = None
        self.image_texture = None
        self.first_frame_flag = True

    def cv2_img_callback(self, data):
        '''
        Callback function that receives the first frame from an Image topic and updates the frame sizing within the GUI.
        '''
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.first_frame_flag:
                self.first_frame_flag = False
                self.frame_size = self.frame.shape[::-1][1::]
                self.size = self.frame_size
        except CvBridgeError as e:
            print(e)

    def update_frames(self, t):
        '''
        Function that updates any incoming frames from an image topic and encodes them accordingly.
        '''
        if self.image_texture is None:
            # Create the texture
            self.image_texture = Texture.create(size=self.frame_size, colorfmt='bgr')
            # Kivy's y coordinate is flipped compared to cv2
            self.image_texture.flip_vertical()
        try:
            # Change the cv2 frame to a buffer to feed to GPU
            self.buf = self.frame.reshape(-1)
            self.image_texture.blit_buffer(self.buf, colorfmt='bgr')
            self.buf = None
        except AttributeError as e:
            # if e is none, means frame not here yet so just wait for next cycle
            return
        
        # call for texture updates once ready:
        self.dispatch('on_texture_update')

    def on_texture_update(self):
        # Update the image widget 
        self.texture = self.image_texture
        self.size = self.frame_size
        self.canvas.ask_update()


if __name__ == '__main__':
    from kivy.app import App
    rospy.init_node("KivyImageTest")

    class TempApp(App):
        def build(self):
            return ROSImageWidget()

    TempApp().run()