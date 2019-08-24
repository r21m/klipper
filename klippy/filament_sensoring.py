import homing, kinematics.extruder
import toolhead, gcode, heater, logging
#generic
import serial  #pip install pyserial
import os


#[filament_sensoring]
#port = /dev/serial/by-id/usb-Arduino_LLC_Arduino_Leonardo-if00
#pat9125_use_axis = X
#counting = 1,1,1,1,1,1
#pat9125_mm_factor = 0.02,0.02,0.02,0.02,0.02,0.02
#invert_finda = False
#calibration_lenght = 100

class Filament_sensoring:
    def __init__(self,config):
            #objects
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        self.heater = None
        self.gcode_id = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
                #serial parameters
        self.serial_port = config.get('serial_port')
        self.serial_speed = self.config.getint('serial_speed', 115200)
        self.serial_timeout = self.config.getfloat('serial_timeout', 0.01)
            #finda
        self.invert_finda = self.config.getboolean('invert_finda', False)
            #pat9125
        count = config.get('counting')
        self.counting = count.split(',')
        pat9125_mm_factors = config.get('pat9125_mm_factor')
        self.pat9125_mm_factor = pat9125_mm_factors.split(',')
        #self.pat9125_use_axis = config.get('pat9125_use_axis', 'Y')
        self.pat9125_resolution_x = config.getint('pat9125_resolution_x', 240)
        self.pat9125_resolution_y = config.getint('pat9125_resolution_y', 240)
        #if not ('X') or ('Y') in self.pat9125_use_axis:
        #    raise homing.EndstopError('Filament sensoring: Use X or Y axis!')
        
        self.calibration_lenght = self.config.getfloat('calibration_lenght', 100)
        self.calibration_feed = self.config.getint('calibration_feed', 120)
           #variables
        self.filament_sensoring_data = {}
           #
        self.gcode.register_command('QUERY_FILAMENT_SENSORING', self.cmd_query_filament_sensoring)
    def handle_ready(self):
            self.connect()
            self.init_board()
            self.reactor.register_timer(self.filament_sensoring_callback,self.reactor.NOW)
        return

    def connect(self):
        try:
            self.sens_serial = serial.Serial(self.serial_port,
                                             self.serial_speed,
                                             timeout = self.serial_timeout)
            return True
        except serial.SerialException:
            raise homing.EndstopError('Filament sensoring: connection exception')
        
    def init_board(self):
        for q in range(6):
            self.reset_pat(q)
            
    def reset_pat(self,num):
        cmd = ('R%i'%(num))
        self.send_cmd(cmd)
        
    def set_resolution(self,axis,res):
        cmd = ('%s%i'%(axis,num))
        self.send_cmd(cmd)

    def send_cmd(self,cmd):
        self.sens_serial.write("%s\n"%cmd)
        return
 
    def read_data(self):
        raw_data = sens_serial.readline()
        raw_data = raw_data[:-2:]
        raw_data = (raw_data.decode('utf-8'))
        raw_data = raw_data.split(',')
        out = []
        for num in raw_data:
            if num is not (''):
                try:
                    out.append(int(num))
                except ValueError:
                    out = []
                    break
        if (len(out)) < 6:
            out = []
        return out        

    def assign_data(self,in_list):
        val_dict = {}
        try:
            tool_num = in_list[0]
            mark = ('T%s' %tool_num)
            
            in_list[2] = in_list[2] * self.counting[tool_num]
            in_list[3] = in_list[3] * self.counting[tool_num]
            val_dict['F'] = bool(in_list[1]) ^ self.invert_finda 
            val_dict['X'] = in_list[2]
            val_dict['Y'] = in_list[3]
            val_dict['B'] = in_list[4]
            val_dict['S'] = in_list[5]
            val_dict['Xmm'] = round((in_list[2] * self.pat9125_mm_factor[tool_num]),1)
            val_dict['Ymm'] = round((in_list[3] * self.pat9125_mm_factor[tool_num]),1)
        except KeyError or TypeError or IndexError:
            val_dict = None    
        self.filament_sensoring_data[mark] = val_dict
        
    def wait_for_finish_scripts(self):
        self.toolhead.wait_moves()
        
    def get_print_time(self):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        return print_time

    def reactor_pause(self,time):
        self.reactor.pause((self.reactor.monotonic()) + time)
        
        #callback, stats
    def self.cmd_query_filament_sensoring(self):
        self.gcode.respond_info('Filament sensoring:')
        self.gcode.respond_info(self.filament_sensoring_data)
        return
    
    def cmd_calibrate(self, e_num):
        self.reset_pat(e_num)
        self.cmd_Tn(e_num)
        self.reactor_pause(1)
        self.gcode.run_script_from_command(
            "G1 E%.5f F%d\n"% (self.calibration_lenght,self.calibration_feed))
        self.wait_for_finish_scripts()
        
    def cmd_Tn(self, tool):
        self.toolhead.set_extruder(self.extruder[tool])
        self.gcode.extruder = self.extruder[tool]
        self.gcode.reset_last_position()
        self.gcode.extrude_factor = 1.
        self.gcode.base_position[3] = self.gcode.last_position[3]
        self.gcode.actual_tool = tool
        
    def filament_sensoring_callback(self,eventtime):
        data = self.read_data()
        assign_data(data)
        self.filament_sensoring_data['print_time'] = self.get_print_time()
        return eventtime + 1

    def stats(self, eventtime):
        return False, ' Filament sensoring %s: '%(self.filament_sensoring_data)
 


        
