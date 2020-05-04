from __future__ import print_function

import os

import rosparam
import rospy
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (QGridLayout, QHBoxLayout, QLabel,
                                         QLineEdit, QScrollArea, QWidget)
from qt_gui.plugin import Plugin


class ROSParamPlugin(Plugin):

    def __init__(self, context):
        super(ROSParamPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ROSParamPlugin')

        # Setyup UI
        widget = QWidget()

        scroll = QScrollArea()
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)

        context.add_widget(scroll)

        self._form = QGridLayout()
        self._form.setAlignment(Qt.AlignTop)
        self._form.setColumnStretch(0, 1)
        self._form.setColumnStretch(1, 2)
        widget.setLayout(self._form)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._params_map = {}
        self.add_search()
        self.add_params()

        self._timer = rospy.Timer(rospy.Duration(0.1), self.update_loop)

    def add_search(self):
        label = QLabel("Search")
        label.setStyleSheet("font-weight: bold")
        self._form.addWidget(label, 0, 0)
        line = QLineEdit("")
        line.textChanged.connect(self.update_search)
        self._form.addWidget(line, 0, 1)

    def update_search(self, text):
        for idx, param in enumerate(self._params_map.keys(), 1):
            show = text in param
            self._form.itemAtPosition(idx, 0).widget().setVisible(show)
            self._form.itemAtPosition(idx, 1).widget().setVisible(show)

    def add_params(self):
        for idx, param in enumerate(sorted(rosparam.list_params('')), 1):
            self._form.addWidget(QLabel(param), idx, 0)
            line = QLineEdit(param)
            self._form.addWidget(line, idx, 1)
            line.textChanged.connect(lambda text: self.set_param(param, text))
            self._params_map[param] = line
            self.update_param(param)

    def set_param(self, param, value):
        if value != "":
            rosparam.set_param(param, value)

    def update_param(self, param):
        value = rosparam.get_param(param)
        widget = self._params_map[param]

        if type(value) != str:
            value = str(value)

        if widget.text() != value:
            widget.setText(value)

    def update_loop(self, evt):
        for a in self._params_map.keys():
            self.update_param(a)

    def shutdown_plugin(self):
        self._timer.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
