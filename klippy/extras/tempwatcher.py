#
#
# 
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import fan
import logging

KELVIN_TO_CELCIUS = -273.15
AMBIENT_TEMP = 25.

class tempWatch:
    def __init__(self, config):
        self.alias = config.get_name().split()[1]
        self.printer = config.get_printer()        
        self.min_temp = config.getfloat('min_temp_adc',5)
        self.max_temp = config.getfloat('max_temp_adc',80)      
        self.sensor = self.printer.lookup_object('heater').setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.speed_delay = self.sensor.get_report_time_delta()
        self.last_temp = 0.
        self.last_temp_time = 0.
        self.min_temp_watch = config.getfloat('min_temp_watch')
        self.max_temp_watch = config.getfloat('max_temp_watch')
        self.min_temp_watch_warn = config.getfloat('min_temp_watch_warn', self.min_temp_watch+10)
        self.max_temp_watch_warn = config.getfloat('max_temp_watch_warn', self.max_temp_watch-10)
                
    def temperature_callback(self, read_time ,temp):
        self.last_temp = temp
        if self.max_temp_watch < temp:
            msg = "%s: Temperature watch: overheat temp=%.1f" % (self.alias, self.last_temp,)
            logging.error(msg)
            self.printer.invoke_shutdown(msg)
            
        elif self.min_temp_watch > temp:
            msg = "%s: Temperature watch: cold temp=%.1f" % (self.alias, self.last_temp,)
            logging.error(msg)
            self.printer.invoke_shutdown(msg)
                        
    def stats(self, eventtime):                 
        return False, '%s: temp=%.1f' % (
        self.alias, self.last_temp)
                                  
def load_config_prefix(config):
    return tempWatch(config)
