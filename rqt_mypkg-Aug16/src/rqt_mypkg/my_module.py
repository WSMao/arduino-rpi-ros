import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from i_robot.msg import *
#from i_robot.msg import mix_irimu_msg_msg

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        self.sonar()
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'i_robot.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('MyPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    ##################################################################################  design  ###########################################################################################################

        self._widget.shoulder.valueChanged[int].connect(self.shoulderCB)

        self._widget.arm.valueChanged[int].connect(self.armCB)

        self._widget.hand.valueChanged[int].connect(self.handCB)

    def shoulderCB(self,value):
        self._widget.lcdNumber_3.display(value)

    def armCB(self, value):
        self._widget.lcdNumber.display(value)

    def handCB(self, value):
        self._widget.lcdNumber_2.display(value)

    def sonarCB(self, msg):
        global apple
        """
        :type msg: ir_msg
        """
        apple = msg.ir_analog[0]
        self._widget.lcdNumber.display(apple)


    def ir(self):
        self.ir_sub =rospy.Subscriber('ir_msg',ir_msg,self.irCB)
    #apple = 0
    ir = []
    def irCB(self,msg):
        global apple
        global ir
        """
        :type msg: ir_msg
        """
        #apple = msg.ir_analog[0]
        ir[0] = msg.ir_analog[0]
        ir[1] = msg.ir_analog[1]
        ir[2] = msg.ir_analog[2]
        ir[3] = msg.ir_analog[3]
        ir[4] = msg.ir_analog[4]
        #self._widget.lcdNumber.display(apple)


    def sonar(self):
        self.sonar_sub =rospy.Subscriber('/sonar_msg',sonar_msg,self.sonarCB)














    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
