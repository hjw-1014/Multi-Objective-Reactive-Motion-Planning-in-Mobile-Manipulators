import os
import rospy
import rospkg

from argparse import ArgumentParser

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QTextCursor, QTextCharFormat, QBrush, QColor
from python_qt_binding.QtWidgets import QWidget, QDialogButtonBox
from urdf_parser_py.urdf import URDF


from iiwas_srv.srv import *


class IiwasManager(Plugin):
    def __init__(self, context):
        super(IiwasManager, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Iiwas Manager')

        # Add argument(s) to the parser.
        parser = ArgumentParser()
        parser.add_argument("-v", "--verbose", action="store_true",
                            dest="verbose",
                            help="Put plugin in verbose mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if args.verbose:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('iiwas_rqt'), 'resource', 'IiwasManager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('IiwasManagerUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._set_iw_services()

        self._connect_services_to_buttons()
        self._get_joint_limits()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def _set_iw_services(self):
        cancel_srv_name = 'cancel_motion'
        ptp_srv_name = 'ptp'
        light_srv_name = 'set_blue_light'
        handguiding_srv_name = 'start_handguiding'
        position_control_srv_name = 'start_position_control'

        b_ns = '/iiwa_back/'

        self._cancel_srv_back = rospy.ServiceProxy(b_ns + cancel_srv_name, CancelMotion, persistent=True)
        self._ptp_srv_back = rospy.ServiceProxy(b_ns + ptp_srv_name, PTP, persistent=True)
        self._light_srv_back = rospy.ServiceProxy(b_ns + light_srv_name, SetBlueLight, persistent=True)
        self._handguiding_srv_back = rospy.ServiceProxy(b_ns + handguiding_srv_name, StartHandguiding, persistent=True)
        self._position_control_srv_back = rospy.ServiceProxy(b_ns + position_control_srv_name, StartPositionControl,
                                                        persistent=True)

        f_ns = '/iiwa_front/'

        self._cancel_srv_front = rospy.ServiceProxy(f_ns + cancel_srv_name, CancelMotion, persistent=True)
        self._ptp_srv_front = rospy.ServiceProxy(f_ns + ptp_srv_name, PTP, persistent=True)
        self._light_srv_front = rospy.ServiceProxy(f_ns + light_srv_name, SetBlueLight, persistent=True)
        self._handguiding_srv_front = rospy.ServiceProxy(f_ns + handguiding_srv_name, StartHandguiding, persistent=True)
        self._position_control_srv_front = rospy.ServiceProxy(f_ns + position_control_srv_name, StartPositionControl,
                                                             persistent=True)

    def _get_joint_limits(self):
        robot = URDF.from_parameter_server()

        for robot_prefix in ['F', 'B']:
            for i in range(7):
                index = str(i+1)
                joint_name = robot_prefix + '_joint_' + index
                limit = robot.joint_map[joint_name].limit

                joint_controller_name = 'controller_' + robot_prefix + '_' + index
                joint_controller = getattr(self._widget, joint_controller_name)
                joint_controller.set_joint_limit(limit)

    def _execute_service(self, service, request):
        try:
            res = service.call(request)

            c = self._widget.textBrowser.textCursor()

            if not res.success:
                format = QTextCharFormat()
                format.setForeground(QBrush(QColor("red")))
                c.setCharFormat(format)
            else:
                format = QTextCharFormat()
                format.setForeground(QBrush(QColor("black")))
                c.setCharFormat(format)

            c.insertText(res.msg + '\n')
            c.movePosition(QTextCursor.End)

            self._widget.textBrowser.setTextCursor(c)
        except:
            c = self._widget.textBrowser.textCursor()
            format = QTextCharFormat()
            format.setForeground(QBrush(QColor("red")))
            c.setCharFormat(format)
            c.insertText('Service Error\n')
            c.movePosition(QTextCursor.End)

            self._widget.textBrowser.setTextCursor(c)


    def _cancel_f_cb(self):
        request = CancelMotionRequest()
        self._execute_service(self._cancel_srv_front, request)

    def _cancel_b_cb(self):
        request = CancelMotionRequest()
        self._execute_service(self._cancel_srv_back, request)

    def _handguiding_f_cb(self):
        request = StartHandguidingRequest()
        self._execute_service(self._handguiding_srv_front, request)

    def _handguiding_b_cb(self):
        request = StartHandguidingRequest()
        self._execute_service(self._handguiding_srv_back, request)

    def _position_f_cb(self):
        mode = 2 if self._widget.impedance_f.isChecked() else 0
        request = StartPositionControlRequest(mode=mode)
        self._execute_service(self._position_control_srv_front, request)

    def _position_b_cb(self):
        mode = 2 if self._widget.impedance_b.isChecked() else 0
        request = StartPositionControlRequest(mode=mode)
        self._execute_service(self._position_control_srv_back, request)

    def _ptp_reset_cb(self, robot_prefix):
        for i in range(7):
            index = str(i + 1)
            joint_controller_name = 'controller_' + robot_prefix + '_' + index
            joint_controller = getattr(self._widget, joint_controller_name)
            joint_controller.setValue(0.0)

    def _ptp_reset_f_cb(self):
        self._ptp_reset_cb('F')

    def _ptp_reset_b_cb(self):
        self._ptp_reset_cb('B')

    def _ptp_accept_cb(self, robot_prefix):
        request = PTPRequest()
        for i in range(7):
            index = str(i + 1)
            joint_controller_name = 'controller_' + robot_prefix + '_' + index
            joint_controller = getattr(self._widget, joint_controller_name)

            value = joint_controller.value()
            request.goal.append(value)
        srv = self._ptp_srv_front if robot_prefix == 'F' else self._ptp_srv_back

        self._execute_service(srv, request)

    def _ptp_apply_b_cb(self):
        self._ptp_accept_cb('B')

    def _ptp_apply_f_cb(self):
        self._ptp_accept_cb('F')

    def _set_led_f_cb(self):
        request = SetBlueLightRequest()
        request.on = self._widget.checkBox_F.isChecked()
        self._execute_service(self._light_srv_front, request)

    def _set_led_b_cb(self):
        request = SetBlueLightRequest()
        request.on = self._widget.checkBox_B.isChecked()
        self._execute_service(self._light_srv_back, request)

    def _connect_services_to_buttons(self):
        self._widget.cancel_f.clicked[bool].connect(self._cancel_f_cb)
        self._widget.cancel_b.clicked[bool].connect(self._cancel_b_cb)

        self._widget.handguiding_f.clicked[bool].connect(self._handguiding_f_cb)
        self._widget.handguiding_b.clicked[bool].connect(self._handguiding_b_cb)

        self._widget.position_f.clicked[bool].connect(self._position_f_cb)
        self._widget.position_b.clicked[bool].connect(self._position_b_cb)

        self._widget.buttonBox_F.button(QDialogButtonBox.Reset).clicked.connect(self._ptp_reset_f_cb)
        self._widget.buttonBox_B.button(QDialogButtonBox.Reset).clicked.connect(self._ptp_reset_b_cb)

        self._widget.buttonBox_F.button(QDialogButtonBox.Apply).clicked.connect(self._ptp_apply_f_cb)
        self._widget.buttonBox_B.button(QDialogButtonBox.Apply).clicked.connect(self._ptp_apply_b_cb)

        self._widget.set_led_F.clicked[bool].connect(self._set_led_f_cb)
        self._widget.set_led_B.clicked[bool].connect(self._set_led_b_cb)



