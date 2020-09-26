#!/usr/bin/env python

from rqt_gui_py.plugin import Plugin
#from my_module_widget import MyModuleWidget
from server import Server

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')

        #self._widget = MyModuleWidget()
        self._widget = Server()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._widget.trigger_configuration()
