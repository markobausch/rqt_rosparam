from __future__ import print_function

from pyqtgraph.parametertree import ParameterTree, parameterTypes, Parameter
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QDialog, QDialogButtonBox, QVBoxLayout, QMenuBar


class ParamDialog(QDialog):
    def __init__(self, parm_type, opts=None, *args, **kwargs):
        super(ParamDialog, self).__init__(*args, **kwargs)

        self.type = parm_type
        self.setWindowTitle("New {}".format(parm_type))

        btn_group = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(btn_group)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        param_tree = ParameterTree()

        type_params = {
            "float": [
                {
                    "name": "default",
                    "title": "Default",
                    "type": "float",
                    "value": 0.0,
                },
                {
                    "name": "step",
                    "type": "float",
                    "value": 0.1,
                    "step": 0.1,
                },
                {
                    "name": "suffix",
                    "type": "str",
                    "value": ''
                },
                {
                    "name": "siPrefix",
                    "type": "bool",
                    "value": False
                },
                # {
                #     "name": "dec",
                #     "type": "bool",
                #     "value": False
                # },
                {
                    "name": "value",
                    "type": "float",
                    "value": 0,
                    "step": 0.1,
                },
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
                    ]
                },
            ],
            "bool": [
                {
                    "name": "default",
                    "title": "Default",
                    "type": "bool",
                    "value": False,
                },
            ],
            "str": [
                {
                    "name": "default",
                    "title": "Default",
                    "type": "str",
                    "value": "",
                },
            ],
        }

        # Add default params
        param = type_params.get(self.type, [])
        param.insert(0, {
            "name": "name",
            "title": "Param Path",
            "type": "str",
            "value": "",
        })

        self.param = Parameter.create(name='params', type='group',
                                      children=param)

        param_tree.setParameters(self.param, showTop=False)

        self.param_path.setPlaceholderText("param path")
        self.layout.addWidget(param_tree)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)

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
                limits.child("min").setValue(limit_min),
                limits.child("max").setValue(limit_max),

        self.setMinimumSize(300, 300)

    def get_opts(self):
        opts = {
            "type": self.type,
        }

        for child in self.param.children():
            if isinstance(child, parameterTypes.GroupParameter):
                continue
            opts[child.name()] = child.value()

        if self.type == "float":
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
        super(ParameterTree, self).dragEnterEvent(event)
        if self.itemAt(event.pos()) is None:
            event.ignore()
        else:
            event.accept()

    def dragMoveEvent(self, event):
        super(ParameterTree, self).dragMoveEvent(event)
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

    def dropEvent(self, event):
        if self.itemAt(event.pos()) is None:
            return

        param = event.source().currentItem().param
        index = self.indexAt(event.pos())
        target = self.itemAt(event.pos()).param
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
        menu.addAction("Add Float").triggered.connect(self.add_param("float"))
        menu.addAction("Add Bool").triggered.connect(self.add_param("bool"))
        menu.addAction("Add Str").triggered.connect(self.add_param("str"))
        menu.addAction("Add Group").triggered.connect(self.add_param("group"))

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
            elif change == "context":
                # currently not available
                m = ParamDialog(param.opts["type"], param.opts)
                if m.exec():
                    param.setOpts(**m.get_opts())

    def add_param(self, param_type):
        def create():
            m = ParamDialog(param_type)
            if not m.exec():
                return

            val = m.get_opts()

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
        return create

    def save_settings(self, _plugin_settings, instance_settings):
        instance_settings.set_value("params", self.param.saveState())

    def restore_settings(self, _plugin_settings, instance_settings):
        params = instance_settings.value("params")
        if params is not None:
            self.param.restoreState(params)
