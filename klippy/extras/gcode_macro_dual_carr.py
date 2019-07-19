# Add ability to define custom g-code macros
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Carriage depend macros

import homing, kinematics.extruder
import toolhead

class GCodeMacroDualCarr:
    def __init__(self,config):
        self.alias = config.get_name().split()[1].upper()
        self.script0 = config.get('gcode_carr0')
        self.script1 = config.get('gcode_carr1')
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        try:
            self.gcode.register_command(self.alias, self.cmd, desc=self.cmd_desc)
        except self.gcode.error as e:
            raise config.error(str(e))
        self.in_script = False
    cmd_desc = "G-Code macro"
    def cmd(self, params):
        self.toolhead = self.printer.lookup_object('toolhead')
        kin = self.toolhead.get_kinematics()
        self.active_carr = kin.get_active_carr()
        if self.in_script:
            raise self.gcode.error("Macro %s called recursively" % (self.alias,))
        self.in_script = True
        try:
            if self.active_carr == 0:
                self.gcode.run_script_from_command(self.script0)
            elif self.active_carr == 1:
                self.gcode.run_script_from_command(self.script1)
        finally:
            self.in_script = False

def load_config_prefix(config):
    return GCodeMacroDualCarr(config)
