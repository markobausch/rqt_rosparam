from __future__ import absolute_import, print_function

import rospy
from python_qt_binding.QtWidgets import (QDialog, QDialogButtonBox, QLabel,
                                         QMenuBar, QVBoxLayout)
from qt_gui.plugin import Plugin

from .pyqtgraph.parametertree import Parameter, ParameterTree, parameterTypes

# Used to prevent exceptions on restoreState
CONFIG_VERSION = 1.0


TYPE_PARAMS = {
    "float": [
        {
            "name": "default",
            "title": "Default",
            "type": "float",
            "value": 0.0,
            "tip": "standard value, used by the yellow reset button",
        },
        {
            "name": "step",
            "type": "float",
            "value": 0.1,
            "step": 1,
            "dec": True,
            "tip": "increment when using the arrows or scroll wheel",
        },
        {
            "name": "suffix",
            "type": "str",
            "value": "",
            "tip": "show a suffix like '100 m'",
        },
        {
            "name": "siPrefix",
            "type": "bool",
            "value": False,
            "tip": "shows '1k' instead of '1000'",
        },
        # {
        #     "name": "dec",
        #     "type": "bool",
        #     "value": False,
        # },
        {
            "name": "limits",
            "type": "group",
            "children": [
                {
                    "name": "enable",
                    "title": "Enable limits",
                    "type": "bool",
                    "value": False,
                },
                {
                    "name": "min",
                    "type": "float",
                    "value": 0,
                },
                {
                    "name": "max",
                    "type": "float",
                    "value": 10,
                },
            ],
        },
    ],
    "int": [
        {
            "name": "default",
            "title": "Default",
            "type": "int",
            "value": 0,
            "tip": "standard value, used by the yellow reset button",
        },
        {
            "name": "step",
            "type": "int",
            "value": 1,
            "step": 1,
            "dec": True,
            "tip": "increment when using the arrows or scroll wheel",
        },
        {
            "name": "suffix",
            "type": "str",
            "value": "",
            "tip": "show a suffix like '100 m'",
        },
        {
            "name": "siPrefix",
            "type": "bool",
            "value": False,
            "tip": "shows '1k' instead of '1000'",
        },
        # {
        #     "name": "dec",
        #     "type": "bool",
        #     "value": False,
        # },
        {
            "name": "limits",
            "type": "group",
            "children": [
                {
                    "name": "enable",
                    "title": "Enable limits",
                    "type": "bool",
                    "value": False,
                },
                {
                    "name": "min",
                    "type": "int",
                    "value": 0,
                },
                {
                    "name": "max",
                    "type": "int",
                    "value": 10,
                },
            ],
        },
    ],
    "bool": [
        {
            "name": "default",
            "title": "Default",
            "type": "bool",
            "value": False,
            "tip": "standard value, used by the yellow reset button",
        },
    ],
    "str": [
        {
            "name": "default",
            "title": "Default",
            "type": "str",
            "value": "",
            "tip": "standard value, used by the yellow reset button",
        },
    ]
}

HELP_TEXT = {
    "group": "With a group you can simplify your param structure. "
             "The group name is used as prefix for the children of the group. "
             "=> /<group_name>/<child_name>"
}


class ParamDialog(QDialog):
    def __init__(self, parm_type, opts=None, *args, **kwargs):
        super(ParamDialog, self).__init__(*args, **kwargs)

        self.type = parm_type
        self.setWindowTitle("New {}".format(parm_type))

        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        param_tree = ParameterTree()

        help_label = QLabel(HELP_TEXT.get(
            parm_type, "Hover over items for more information"))

        help_label.setWordWrap(True)

        layout = QVBoxLayout()
        layout.addWidget(help_label)
        layout.addWidget(param_tree)
        layout.addWidget(button_box)

        self.setLayout(layout)

        # Add default params
        param = TYPE_PARAMS.get(self.type, []).copy()
        param.insert(0, {
            "name": "name",
            "title": "param name",
            "type": "str",
            "tip": "name of the parameter, like: 'test/param'",
            "value": "",
        })

        self.param = Parameter.create(name='params', type='group',
                                      children=param)

        param_tree.setParameters(self.param, showTop=False)

        # populate parm tree for editing
        if opts is not None:
            for child in self.param.children():
                name = child.name()
                if name in opts:
                    child.setValue(opts[name])

            if "limits" in opts:
                limit_min, limit_max = opts["limits"]
                limits = self.param.child("limits")
                limits.child("enable").setValue(True)
                limits.child("min").setValue(limit_min)
                limits.child("max").setValue(limit_max)

        self.setMinimumSize(300, 300)

    def get_opts(self):
        opts = {
            "type": self.type,
        }

        for child in self.param.children():
            # prevent iterating over group (they don't have a value)
            if isinstance(child, parameterTypes.GroupParameter):
                continue
            opts[child.name()] = child.value()

        # removing leading & trailing /
        opts["name"] = opts["name"].strip("/").replace(" ", "_")

        if self.type in ["float", "int"]:
            limits = self.param.child("limits")
            if limits.child("enable").value():
                opts["limits"] = (
                    limits.child("min").value(),
                    limits.child("max").value(),
                )

        return opts


class Tree(ParameterTree):
    """"Custom Paramtree to support drag and drop reorder"""

    def dragEnterEvent(self, event):
        super(Tree, self).dragEnterEvent(event)
        if self.itemAt(event.pos()) is None:
            event.ignore()
        else:
            event.accept()

    def dragMoveEvent(self, event):
        super(Tree, self).dragMoveEvent(event)
        if self.itemAt(event.pos()) is None:
            event.ignore()
        else:
            item = event.source().currentItem().param
            target = self.itemAt(event.pos()).param

            # check if draged item is parent of target
            if item.childPath(target):
                event.ignore()
                return

            # prevent name collision
            name = item.name()
            for child in target.children():
                if child.name() == name:
                    event.ignore()
                    return

            event.accept()

    def dropEvent(self, ev):
        if self.itemAt(ev.pos()) is None:
            return

        param = ev.source().currentItem().param
        index = self.indexAt(ev.pos())
        target = self.itemAt(ev.pos()).param
        if param == target:
            return
        new_parent = target.parent()

        # remove item
        param.remove()

        if isinstance(target, parameterTypes.GroupParameter):
            target.addChild(param.opts)
        else:
            # create a new item from the old opts
            new_parent.insertChild(index.row(), param.opts)


class ROSParamPlugin(Plugin):

    def __init__(self, context):
        super(ROSParamPlugin, self).__init__(context)

        self.setObjectName('rqt_rosparam')
        param_tree = Tree()
        param_tree.setObjectName("rqt_rosparamUI")
        param_tree.setWindowTitle("rqt_rosparam")

        if context.serial_number() > 1:
            param_tree.setWindowTitle(param_tree.windowTitle() + (
                " ({:d})".format(context.serial_number())))

        self.param = parameterTypes.GroupParameter(name="Root Group")

        menu = QMenuBar(param_tree)
        for param_type in ["float", "int", "bool", "str", "group"]:
            menu.addAction("Add {}".format(param_type)).triggered.connect(
                self.add_param_dialog(param_type))

        menu.addAction("Load all params").triggered.connect(self.load_params)

        param_tree.setParameters(self.param, showTop=False)

        context.add_widget(param_tree)

        self.param.sigTreeStateChanged.connect(self.on_change)

    def on_change(self, _param, changes):
        for param, change, data in changes:
            path = self.param.childPath(param)
            if path is not None:
                child_name = "/".join(path)
            else:
                child_name = param.name()

            if change == "value":
                rospy.set_param(child_name, data)
                print("Set '{}' to '{}'".format(child_name, data))
            elif change == "contextMenu":
                dialog = ParamDialog(param.opts["type"], param.opts)
                if dialog.exec_():
                    param.setOpts(**dialog.get_opts())

    def add_param_dialog(self, param_type):
        def create():
            dialog = ParamDialog(param_type)
            if not dialog.exec_():
                return

            val = dialog.get_opts()

            self.add_param(val)
        return create

    def add_param(self, val):
        default_params = {
            "removable": True,
            "renamable": True,
            "movable": True,
            "context": {
                "edit": "Edit"
            }
        }

        opts = default_params.copy()
        opts.update(val)
        self.param.addChild(opts, autoIncrementName=True)

    def load_params(self):

        def parse_params(data, path=""):
            for key, value in data.items():
                # skip default ros params
                if path == "" and key in ["run_id", "roslaunch", "rosversion",
                                          "rosdistro"]:
                    continue

                current_path = "{}/{}".format(path, key)

                if isinstance(value, dict):
                    parse_params(value, current_path)
                elif current_path not in self.param.names:
                    type_name = type(value).__name__
                    val = {
                        "name": current_path.strip("/"),
                        "type": type_name,
                        "default": value,
                        "value": value,
                    }

                    self.add_param(val)

        params = rospy.get_param("")
        parse_params(params)

    def save_settings(self, _plugin_settings, instance_settings):
        instance_settings.set_value("params", self.param.saveState())
        instance_settings.set_value("version", CONFIG_VERSION)

    def restore_settings(self, _plugin_settings, instance_settings):
        params = instance_settings.value("params")
        version = instance_settings.value("version")
        if version == CONFIG_VERSION and params is not None:
            self.param.restoreState(params)
