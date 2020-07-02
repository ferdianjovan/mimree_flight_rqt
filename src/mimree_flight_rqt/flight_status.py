import os
from copy import deepcopy

import mavros_msgs.msg as mavros_msgs
import rospkg
import rospy
import sensor_msgs.msg as sensor_msgs
from mavros_msgs.srv import CommandLong
from python_qt_binding import QtWidgets, loadUi
from python_qt_binding.QtCore import Qt, Signal
from qt_gui.plugin import Plugin
from rosplan_dispatch_msgs.msg import (ActionDispatch, ActionFeedback,
                                       EsterelPlan)


class LabelledNumber(QtWidgets.QWidget):
    def __init__(self, labelText, *args, **kwargs):

        super(LabelledNumber, self).__init__(*args, **kwargs)

        layout = QtWidgets.QHBoxLayout()

        self.label = QtWidgets.QLabel(text=labelText)
        self.label.setAlignment(Qt.AlignHCenter)
        self.content = QtWidgets.QLabel()

        layout.addWidget(self.label)
        layout.addWidget(self.content)

        self.setLayout(layout)

    def setValue(self, value):
        self.content.setText("{: 3.5f}".format(value))


class VectorDisplay(QtWidgets.QWidget):
    def __init__(self, *args, **kwargs):
        super(VectorDisplay, self).__init__(*args, **kwargs)

        layout = QtWidgets.QVBoxLayout()

        self.x_widget = LabelledNumber("x:")
        self.y_widget = LabelledNumber("y:")
        self.z_widget = LabelledNumber("z:")

        layout.addWidget(self.x_widget)
        layout.addWidget(self.y_widget)
        layout.addWidget(self.z_widget)

        self.setLayout(layout)

    def display(self, values):
        self.x_widget.setValue(values[0])
        self.y_widget.setValue(values[1])
        self.z_widget.setValue(values[2])


class FlightStatusPlugin(Plugin):
    # General Box
    vehicleNameSignal = Signal(str)
    vehicleModeSignal = Signal(str)
    vehicleBatterySignal = Signal(float)
    # Action Box
    nextActionSignal = Signal(str)
    currentActionSignal = Signal(str)
    durationActionSignal = Signal(float)
    # GPS Box
    gpsPositionSignal = Signal(list)

    def __init__(self, context):
        super(FlightStatusPlugin, self).__init__(context)

        ui_file = os.path.join(rospkg.RosPack().get_path('mimree_flight_rqt'),
                               'resource', 'FlightStatus.ui')
        self.setObjectName('FlightStatusWidget')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser
        parser.add_argument("--fcu",
                            dest="fcu",
                            default="hector",
                            help='Flight Control Unit\'s name')
        args = parser.parse_args(context.argv())
        self.fcu_name = args.fcu

        # Extend self with all attributes and children from UI file
        self._widget = QtWidgets.QWidget()
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

        self.vehicleNameSignal.connect(
            self._widget.vehicle_name_content.setText)
        self.vehicleNameSignal.emit(args.fcu.upper())

        self.mode_sub = self.forward_to_signal("/%s/mavros/state" % args.fcu,
                                               mavros_msgs.State,
                                               self.vehicleModeSignal,
                                               lambda msg: msg.mode)
        self.vehicleModeSignal.connect(
            self._widget.vehicle_mode_content.setText)

        # mavros topics
        self.battery_sub = self.forward_to_signal(
            "/%s/mavros/battery" % args.fcu, sensor_msgs.BatteryState,
            self.vehicleBatterySignal, lambda msg: msg.voltage)
        self.vehicleBatterySignal.connect(self._widget.voltage_display.display)

        self.gpsPosition_sub = rospy.Subscriber(
            "/%s/mavros/global_position/global" % args.fcu,
            sensor_msgs.NavSatFix, self.handle_gpsPosition)
        self.gpsPositionSignal.connect(self._widget.position_display.display)

        # rosplan topics
        self.complete_plan = EsterelPlan().nodes
        self.current_action = ActionDispatch()
        self.complete_plan_sub = rospy.Subscriber(
            '/rosplan_parsing_interface/complete_plan', EsterelPlan,
            self._complete_plan_cb)
        self.action_dispatch_sub = rospy.Subscriber(
            '/rosplan_plan_dispatcher/action_dispatch', ActionDispatch,
            self._dispatch_cb)
        self.action_feedback_sub = rospy.Subscriber(
            '/rosplan_plan_dispatcher/action_feedback', ActionFeedback,
            self._feedback_cb)
        self.nextActionSignal.connect(self._widget.next_action_content.setText)
        self.currentActionSignal.connect(self._widget.current_action.setText)
        self.durationActionSignal.connect(self._widget.duration_number.display)

        # mavros service proxies
        rospy.wait_for_service('/%s/mavros/cmd/command' % args.fcu)
        self._arming_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/command' % args.fcu, CommandLong)

        self.last_position_time = rospy.Time()
        self.last_setpoint_time = rospy.Time()

        self.validityChecker = rospy.Timer(rospy.Duration(0.5),
                                           self.check_valid_times)

        self._widget.arm_button.clicked.connect(
            lambda: self.button_box(self._widget.arm_button_box, True))
        self._widget.arm_button_box.accepted.connect(
            lambda: self.arming_disarming_button(True, self._widget.
                                                 arm_button_box))
        self._widget.arm_button_box.rejected.connect(
            lambda: self.button_box(self._widget.arm_button_box, False))
        self._widget.disarm_button.clicked.connect(
            lambda: self.button_box(self._widget.disarm_button_box, True))
        self._widget.disarm_button_box.accepted.connect(
            lambda: self.arming_disarming_button(
                False, self._widget.disarm_button_box))
        self._widget.disarm_button_box.rejected.connect(
            lambda: self.button_box(self._widget.disarm_button_box, False))

    def _complete_plan_cb(self, msg):
        self.complete_plan = msg.nodes

    def _dispatch_cb(self, msg):
        current_action = "WAITING..."
        next_action = "NONE"
        if msg.name != "cancel_action":
            for param in msg.parameters:
                if param.value == self.fcu_name:
                    action_name = msg.name.split("_")[1:]
                    current_action = " ".join(action_name).upper()
                    break
            self.current_action = msg if (
                current_action != "WAITING...") else ActionDispatch()
            reduced_plan = deepcopy(self.complete_plan)
            for node in self.complete_plan:
                if msg.action_id >= node.action.action_id:
                    del reduced_plan[0]
                else:
                    next_fcu_action = False
                    for param in node.action.parameters:
                        if param.value == self.fcu_name:
                            next_fcu_action = True
                            break
                    if next_fcu_action:
                        break
                    else:
                        del reduced_plan[0]
            if len(reduced_plan):
                action_name = reduced_plan[0].action.name.split("_")[1:]
                next_action = " ".join(action_name).upper()
            self.complete_plan = reduced_plan
            self.currentActionSignal.emit(current_action)
            self._widget.current_action.setStyleSheet(
                "background-color: rgb(115, 210, 22)")
        elif (msg.name == "cancel_action") and (
                self.complete_plan == EsterelPlan().nodes):
            self.currentActionSignal.emit("-")
            self.current_action = ActionDispatch()
        self.nextActionSignal.emit(next_action)

    def _feedback_cb(self, msg):
        if (self.current_action != ActionDispatch()) and (
                msg.action_id == self.current_action.action_id):
            if msg.status == "action failed":
                text = self.current_action.name.replace("_", " ").upper()
                text += "\n(FAILED)"
                self.currentActionSignal.emit(text)
                self._widget.current_action.setStyleSheet(
                    "background-color: red")

    def arming_disarming_button(self, arming, button):
        self._arming_proxy(False, 400, 0, float(arming), 21196, 0.0, 0.0, 0.0,
                           0.0, 0.0)
        button.setEnabled(False)

    def button_box(self, button, enabled):
        button.setEnabled(enabled)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.mode_sub.unregister()
        self.battery_sub.unregister()
        self.gpsPosition_sub.unregister()
        self.complete_plan_sub.unregister()
        self.action_dispatch_sub.unregister()
        self.action_feedback_sub.unregister()
        self.validityChecker.shutdown()
        self._arming_proxy.close()

    def forward_to_signal(self, topic, type, signal, converter=lambda x: x):
        return rospy.Subscriber(topic, type,
                                lambda msg: signal.emit(converter(msg)))

    def check_valid_times(self, event):
        if event.current_real - self.last_position_time > rospy.Duration(0.75):
            # Timeout on position
            self._widget.position_validity.setText('TIMEOUT')
            self._widget.position_validity.setStyleSheet(
                "background-color: orange")
        if (self.current_action != ActionDispatch()):
            remaining_duration = rospy.Duration(self.current_action.duration)
            remaining_duration = remaining_duration - rospy.Duration(0.5)
            self.current_action.duration = remaining_duration.to_sec()
            self.durationActionSignal.emit(self.current_action.duration)
            if (self.current_action.duration < 0.0):
                self._widget.duration_number.setStyleSheet(
                    "color: rgb(239, 41, 41);\ngridline-color: rgb(0, 0, 0);")
            else:
                self._widget.duration_number.setStyleSheet(
                    "color: rgb(0, 0, 0);\ngridline-color: rgb(0, 0, 0);")
        else:
            self.durationActionSignal.emit(0.0)
            self._widget.duration_number.setStyleSheet(
                "color: rgb(0, 0, 0);\ngridline-color: rgb(0, 0, 0);")

    def handle_gpsPosition(self, msg):
        self.last_position_time = rospy.Time.now()
        position = [msg.latitude, msg.longitude, msg.altitude]
        self.forward_vector(position, self.gpsPositionSignal)
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            # Likely invalid posiiton solution, set warning
            self._widget.position_validity.setText('INVALID')
            self._widget.position_validity.setStyleSheet(
                "background-color: red")
        else:
            self._widget.position_validity.setText('CURRENT')
            self._widget.position_validity.setStyleSheet(
                "background-color: green")

    def forward_vector(self, position, signal):
        # signal.emit([getattr(position,e) for e in ['x','y','z']])
        signal.emit([position[0], position[1], position[2]])
