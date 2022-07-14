#!/usr/bin/env python3

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QMessageBox

from std_msgs.msg import String

class RqtTablesDemo(Plugin):

    def __init__(self, context):
        super(RqtTablesDemo, self).__init__(context)
        rospy.loginfo('Initializing rqt_tables_demo, have a happy pick, place and move and more!')

        self.setObjectName('RqtTablesDemo')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file (xml description of the gui window created with qtcreator)
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_tables_demo'), 'config', 'rqt_tables_demo.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_tables_demo.ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # class variables
        # self.foo = None

        # publications
        # self.object_mesh_pub = rospy.Publisher('/grasp_editor/update_object_mesh', String, queue_size=1)

        # parameters
        # obj_pkg_name = rospy.get_param('obj_pkg_name', 'mobipick_gazebo')

        ## make a connection between the qt objects and this class methods
        self._widget.cmdHi.clicked.connect(self.foo)

        context.add_widget(self._widget)
        rospy.loginfo('init finished')
        # end of constructor

    # ::::::::::::::  class methods

    def foo(self):
        rospy.loginfo('Hi!')
