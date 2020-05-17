from __future__ import print_function

from pyqtgraph.parametertree import ParameterTree, parameterTypes
import rosparam
from qt_gui.plugin import Plugin


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

        param_tree = ParameterTree()

        self.param = ParamGroup(name="Root Group")

        param_tree.setParameters(self.param, showTop=False)

        context.add_widget(param_tree)

        self.param.sigTreeStateChanged.connect(self.on_change)

    @staticmethod
    def on_change(_param, changes):
        for param, change, data in changes:
            param = param.name()

            if change == "value":
                rosparam.set_param(param, data)

    def save_settings(self, _plugin_settings, instance_settings):
        instance_settings.set_value("params", self.param.saveState())

    def restore_settings(self, _plugin_settings, instance_settings):
        params = instance_settings.value("params")
        self.param.restoreState(params)
