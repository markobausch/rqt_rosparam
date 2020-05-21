from __future__ import print_function

from pyqtgraph.parametertree import ParameterTree, parameterTypes
import rospy
from qt_gui.plugin import Plugin


class ParamDialog(QDialog):
    def __init__(self, parm_type, *args, **kwargs):
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
                    "name": "value",
                    "title": "Default",
                    "type": "bool",
                    "value": False,
                },
            ],
            "str": [
                {
                    "name": "value",
                    "title": "Default",
                    "type": "str",
                    "value": False,
                },
            ],
        }

        self.param = Parameter.create(name='params', type='group',
                                      children=type_params.get(self.type, []))

        param_tree.setParameters(self.param, showTop=False)
        self.param_path = QLineEdit()

        # get topics
        self.param_path.setCompleter(QCompleter(["/test/test", "/param2/float",
                                                 "apples", "banan"]))

        self.param_path.setPlaceholderText("param path")
        self.layout.addWidget(self.param_path)
        self.layout.addWidget(param_tree)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)

    def get_opts(self):
        opts = {
            "name": self.param_path.text(),
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
        print(target, new_parent)

        # remove item
        param.remove()

        if isinstance(target, parameterTypes.GroupParameter):
            print("True")
            target.addChild(param.opts)
        else:
            # create a new item from the old opts
            new_parent.insertChild(index.row(), param.opts)


class ParamGroup(parameterTypes.GroupParameter):
    """"Group for parameters with add button to add new parameters"""

    def __init__(self, **opts):
        self.types = {
            'string': {
                "name": "string_param",
                "type": "str",
                "value": "",
            },
            'float': {
                "name": "float_param",
                "type": "float",
                "step": 0.1,
                "value": 0,
            },
            'bool': {
                "name": "bool_param",
                "type": "bool",
                "value": False,
            }
        }

        opts['type'] = 'group'
        opts['addText'] = "New Param"
        opts['addList'] = self.types.keys()

        super(ParamGroup, self).__init__(**opts)

    def addNew(self, typ=None):
        if typ is None:
            return

        val = self.types[typ]

        default_params = {
            "removable": True,
            "renamable": True,
        }

        opts = default_params.copy()
        opts.update(val)

        self.addChild(opts, autoIncrementName=True)


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

        self.param = ParamGroup(name="Root Group")

        menu = self.menuBar()
        menu.addAction("Add Float").triggered.connect(self.add_number)
        menu.addAction("Add Bool").triggered.connect(self.add_bool)
        menu.addAction("Add Str").triggered.connect(self.add_str)
        menu.addAction("Add Group").triggered.connect(self.add_group)

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

    def add_child(self, val):
        default_params = {
            "removable": True,
            "renamable": True,
            "movable": True,
        }

        opts = default_params.copy()
        opts.update(val)
        self.param.addChild(opts, autoIncrementName=True)

    def add_group(self):
        m = ParamDialog("group")
        if m.exec():
            self.add_child(m.get_opts())

    def add_bool(self):
        m = ParamDialog("bool")
        if m.exec():
            self.add_child(m.get_opts())

    def add_str(self):
        m = ParamDialog("str")
        if m.exec():
            self.add_child(m.get_opts())

    def add_number(self):
        m = ParamDialog("float")
        if m.exec():
            self.add_child(m.get_opts())

    def save_settings(self, _plugin_settings, instance_settings):
        instance_settings.set_value("params", self.param.saveState())

    def restore_settings(self, _plugin_settings, instance_settings):
        params = instance_settings.value("params")
        if params is not None:
            self.param.restoreState(params)
