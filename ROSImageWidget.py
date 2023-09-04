#!/usr/bin/python3

from kivy.clock import Clock
from kivy.properties import ListProperty, StringProperty, NumericProperty, BooleanProperty
from kivy.graphics.texture import Texture
from kivy.uix.image import Image as KivyImage

from kivymd.uix.button import MDFillRoundFlatIconButton
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

KV = '''
MDScreen:
    
    ROSImageWidget:
        id: robo_view
        image_topic: "/camera1/image_raw"
        size_hint_x: None
        size_hint_y: None
        pos_hint: {'center_x': 0.5, "center_y": 0.5}
        size: self.texture_size
        canvas.before:
            Color:
                rgba: (53/255, 175/255, 253/255, 0.3)
            RoundedRectangle:
                size: robo_view.size[0] + 30, robo_view.size[1] + 30
                pos: robo_view.pos[0] - 15, robo_view.pos[1] - 15
    
    Label:
        text:"[size=30]Sample Demo of ROSImageWidget:[/size]"
        markup: True
        size: self.texture_size
        pos: (robo_view.pos[0] + robo_view.texture_size[0]/2 - self.width/2, robo_view.pos[1] + self.height/4)
        # pos:robo_view.pos
        outline_width: 2

    MDBoxLayout:
        size_hint_x: None
        size_hint_y: None
        pos_hint_x: None
        pos_hint_y: None
        # size: 500, 100
        adaptive_width: True
        adaptive_height: True
        padding: 10, 10
        spacing: 30
        orientation:'horizontal'
        pos: (robo_view.pos[0] + robo_view.texture_size[0]/2 - self.width/2, robo_view.pos[1] - self.height/2)

        MDFillRoundFlatIconButton:
            text: "Point at camera"
            icon: 'robot-industrial'
            size:self.size
            on_release: app.trigger_move()
        
        MDFillRoundFlatIconButton:
            text: "Change camera"
            icon: 'camera'
            size:self.size
            on_release: 
                app.view_switch = not app.view_switch;
                robo_view.image_topic = ["/camera2/image_raw", "/camera1/image_raw"][app.view_switch]
        
        MDFillRoundFlatIconButton:
            text: "Random movement"
            icon: 'robot-confused'
            size:self.size
            on_release: 
                app.random_go()
'''

class ROSImageWidget(KivyImage):
    '''
    Widget that allows for the visualization of an image topic into a kivy app easily
    '''
    __events__ = ('on_texture_update',)
    frame_size = ListProperty([None,None])
    image_topic = StringProperty("/camera")
    frame_rate = NumericProperty(60)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        self.bridge = CvBridge()
        self.img_updater = Clock.schedule_interval(self.update_frames, 1.0/self.frame_rate)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cv2_img_callback, queue_size=1)
        self.bind(image_topic=self.create_subscriber)
        
        self.frame = None
        self._buffer = None
        self.image_texture = None
        self.first_frame_flag = True

    def create_subscriber(self, *args):
        '''
        Sets up the ROS image subscriber based on the passed Image topic
        '''
        try:
            self.image_sub.unregister()
            self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cv2_img_callback, queue_size=1)
        except Exception as e:
            print(e)

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
        '''
        Once a texture is ready (frame from camera), send to GPU to render into screen
        '''
        self.texture = self.image_texture
        self.size = self.frame_size
        self.canvas.ask_update()


if __name__ == '__main__':
    # Here is a test that will spawn an image into the screen
    from kivymd.app import MDApp
    from kivy.lang import Builder
    rospy.init_node("KivyImageTest")



    class TempApp(MDApp):
        view_switch = BooleanProperty(True)
        pub1 = rospy.Publisher('/move_to_pose', Bool, queue_size=1)
        pub2 = rospy.Publisher('/move_to_random_pose', Bool, queue_size=1)
        def build(self):
            return Builder.load_string(KV)
        
        def trigger_move(self):
            self.pub1.publish(Bool(self.view_switch))

        def random_go(self):
            self.pub2.publish(Bool(True))



    TempApp().run()
