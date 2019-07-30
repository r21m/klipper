# Add ability to define custom g-code macros
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# tool depend macros

import homing, kinematics.extruder,traceback
import toolhead, gcode

class GCodeMacroIfTool:
    def __init__(self,config):
        self.alias = config.get_name().split()[1].upper()
        self.scripts = []
        for k in range (0,15,1):
            self.scripts.append(config.get('gcode_tool' + str(k), ""))
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        try:
            self.gcode.register_command(self.alias, self.cmd, desc=self.cmd_desc)
        except self.gcode.error as e:
            raise config.error(str(e))
        self.in_script = False
        self._actual_tool = 0
    cmd_desc = "G-Code macro if tool"
    def cmd(self, params):
        if self.in_script:
            raise self.gcode.error("Macro %s called recursively" % (self.alias,))
        self.in_script = True
        try:
            self._actual_tool = self.gcode.get_actual_tool()
            self.gcode.run_script_from_command(self.scripts[self._actual_tool])
        finally:
            self.in_script = False

def load_config_prefix(config):
    return GCodeMacroIfTool(config)
