import os
import rospkg
import numpy as np

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class JointController(QWidget):
    def __init__(self, *args):
        super(JointController, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('iiwas_rqt'), 'resource', 'JointController.ui')
        loadUi(ui_file, self)

        self.slider.valueChanged.connect(self._slider_updated)
        self.spin.valueChanged.connect(self._spin_updated)

    def set_joint_limit(self, limit):
        lower = np.rad2deg(limit.lower)
        upper = np.rad2deg(limit.upper)

        self.slider.setMinimum(lower * 100)
        self.slider.setMaximum(upper * 100)

        self.spin.setMinimum(lower)
        self.spin.setMaximum(upper)

    def _slider_updated(self):
        val = self.slider.value() / 100.0
        self.spin.blockSignals(True)
        self.spin.setValue(val)
        self.spin.blockSignals(False)

    def _spin_updated(self):
        val = self.spin.value() * 100
        self.slider.blockSignals(True)
        self.slider.setValue(val)
        self.slider.blockSignals(False)

    def setValue(self, value):
        return self.spin.setValue(value)

    def value(self):
        return np.deg2rad(self.spin.value())
