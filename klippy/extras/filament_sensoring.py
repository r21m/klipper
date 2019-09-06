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

class Filament_sensoring:
    def __init__(self,config):
            #objects
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        self.heater = None
        self.gcode_id = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        reactor = self.printer.get_reactor()
        self.extruder = kinematics.extruder.get_printer_extruders(self.printer)
            #serial parameters
        self.serial_port = config.get('serial_port')
        self.serial_speed = self.config.getint('serial_speed', 115200)
        self.serial_timeout = self.config.getfloat('serial_timeout', 0.02)
            #finda
        self.invert_finda = self.config.getboolean('invert_finda', False)
            #pat9125
        count = config.get('pat9125_counting')
        self.counting = count.split(',')
        pat9125_mm_factors = config.get('pat9125_mm_factor')
        self.pat9125_mm_factor = pat9125_mm_factors.split(',')
        
        self.maxval_t = len(self.pat9125_mm_factor)
        for q in range (self.maxval_t): 
            self.pat9125_mm_factor[q] = float(self.pat9125_mm_factor[q])
            self.counting[q] = float(self.counting[q])
            
        self.pat9125_use_axis = config.get('pat9125_use_axis', 'Y')
        self.pat9125_resolution = config.getint('pat9125_resolution', 240)
        self.pat9125_used = config.getint('pat9125_used', 6)
        #self.pat9125_resolution_y = config.getint('pat9125_resolution_y', 240)
        if not ('X') or not('Y') in self.pat9125_use_axis:
            raise homing.EndstopError("Filament sensoring: Use X or Y axis!")
        self.calibration_lenght = self.config.getfloat('calibration_lenght', 100)
        self.calibration_feed = self.config.getint('calibration_feed', 120)
            #variables
        self.filament_sensoring_data = {}
        self.state = None
        self.t_prev = [0 for x in range(self.maxval_t)]
        self.l_pre = [0 for x in range(self.maxval_t)]
            #
        self.gcode.register_command('QUERY_FS', self.cmd_query_filament_sensoring,desc='')
        self.gcode.register_command('CAL_FS', self.cmd_calibrate_lenght,desc='')
        self.gcode.register_command('RESET_FS', self.cmd_reset,desc='')
        self.filament_sensoring_timer = reactor.register_timer(self.filament_sensoring_callback)
           #scripts
        self.gcode_before_cal = self.config.get('gcode_before_cal', '')
        self.gcode_after_cal = self.config.get('gcode_after_cal', '')
        
    def handle_ready(self):
        self.init_board()
        reactor = self.printer.get_reactor()
        reactor.update_timer(self.filament_sensoring_timer,reactor.NOW)
        self.toolhead = self.printer.lookup_object('toolhead')
        self.restore()
        self.gcode.respond_info("FS: Ready")
        self.state = ('ready')
        return

    def connect(self):
        try:
            self.sens_serial = serial.Serial(self.serial_port,self.serial_speed,timeout = self.serial_timeout)
        except serial.SerialException:
            raise homing.EndstopError('FS: Connection exception')
        
    def init_board(self):
        self.connect() 
        for q in range(6):
            self.reset_pat(q)
        self.set_resolution('X',self.pat9125_resolution)
        self.set_resolution('Y',self.pat9125_resolution)
           
    def reset_pat(self,num):
        cmd = ('R%i'%(num))
        self.send_cmd(cmd)
        
    def reset_board(self):
        self.send_cmd('Q')    
        
    def set_resolution(self,axis,res):
        cmd = ('%s%i'%(axis,res))
        self.send_cmd(cmd)

    def send_cmd(self,cmd):
        self.sens_serial.write("%s\n"%(cmd))
        #self.sens_serial.flush()
        #self.reactor_pause(0.5)
            
    def read_data(self):
        try:
            s = self.sens_serial.readline()
        except serial.SerialException:
           #self.state = ('serial exception')
           self.sens_serial.close()
           self.connect()
           return 
        s = s[:-2:]
        s = s.split(',')

        out = []
        try:
            for val in s:
               out.append(int(val))
        except ValueError:
            return
            
        val_dict = {}            
        if len(out) == (6*self.pat9125_used):
            c = 0
            for rep in range(6):
                tool_num = out[0+c]
                mark = ('T%s' %tool_num)
                val_dict = {}
                if out[5+c] != -1:
                    if ('X') in self.pat9125_use_axis:
                        val_dict['L'] = out[2+c] * int(self.counting[rep])
                        val_dict['Lmm'] = round((val_dict['L'] * self.pat9125_mm_factor[tool_num]),6)
                    if ('Y') in self.pat9125_use_axis:    
                        val_dict['L'] = out[3+c] * int(self.counting[rep])
                        val_dict['Lmm'] = round((val_dict['L'] * self.pat9125_mm_factor[tool_num]),6)
                    
                    val_dict['B'] = out[4+c]
                    val_dict['S'] = out[5+c]
                    val_dict['F'] = bool(out[1+c]) ^ self.invert_finda
                    
                    if (abs(val_dict['L'] - self.l_pre[tool_num]) > 5): 
                        val_dict['M'] = ('MOVE')
                    else:
                        val_dict['M'] = ('STOP')   
                    
                    self.l_pre[tool_num] = val_dict['L'] 
                    self.filament_sensoring_data[mark] = val_dict
                else:
                    self.filament_sensoring_data[mark] = val_dict
                c += 6
            #self.filament_sensoring_data['print_time'] = self.get_print_time()    
            #logging.info(self.filament_sensoring_data)

    def wait_for_finish_scripts(self):
        self.toolhead.wait_moves()
        
    def get_print_time(self):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        return print_time

    def reactor_pause(self,time):
        reactor = self.printer.get_reactor()
        reactor.pause((reactor.monotonic()) + time)
        
        #callback, stats
    def cmd_query_filament_sensoring(self):
        self.gcode.respond_info("FS: data:")
        self.gcode.respond_info(self.filament_sensoring_data)
        return
        
    def cmd_reset(self,params):
        if ('R') in params:
            num = params['R']
            self.reset_pat(num)
            self.gcode.respond_info("FS: Resetting %i" %num)
        return
    
        #Calibrate
    def cmd_calibrate_lenght(self, params): #FS_CALIBRATE E<NUM>
        _len = self.calibration_lenght
        _feed = self.calibration_feed
        store = False
    
        if ('E') in params:
            e_num = params['E']
            self.cmd_Tn(e_num)
        else:    
            e_num = self.gcode.get_active_extruder() #current active
            if e_num is None:
                self.gcode.respond_error('FS: No active extruder')
                return     
        if ('L') in params: _len = int(params['L'])
        if ('F') in params: _feed = int(params['F'])
                              
        self.gcode.respond_info("FS: Calibration begin T:%i L:%i F:%i"%(e_num,_len,_feed))        
        self.gcode.run_script_from_command(self.gcode_before_cal)        
        self.wait_for_finish_scripts()        
        cal_lenght = self.cmd_calibrate(e_num, _len,_feed)
        if (cal_lenght > 1):
            self.gcode.respond_error('FS: Calibrated value is > 1')
            return
            
        self.gcode.respond_info("FS: Calibrated T:%i new:%f old:%f"%(e_num,cal_lenght,self.pat9125_mm_factor[e_num]))
        self.pat9125_mm_factor[e_num] = cal_lenght
        self.gcode.run_script_from_command(self.gcode_after_cal)
        self.store()
        self.wait_for_finish_scripts()
        return
    
    def cmd_calibrate(self, e_num, _len, _feed):
        _len = float(_len)
        cal_lenght = 0.0
        t = ('T%i'%(e_num))
        #self.reset_pat(e_num)
        self.reactor_pause(1)
        old_l = float(self.filament_sensoring_data[t]['L'])
            #select tool and extrude

        self.reactor_pause(1)
        self.gcode.run_script_from_command(
            "M83 \nG92 E0 \nG1 E%.5f F%d\n"% (_len,_feed))
        self.wait_for_finish_scripts()
        self.reactor_pause(1)
        new_l = float(self.filament_sensoring_data[t]['L'])
        cal_lenght = (_len / (new_l - old_l))
        #msg = ("FS: Cal_lenght:%f _len:%i new_l:%i old_l:%i"%(cal_lenght,_len,new_l,old_l))
	#self.gcode.respond_info(msg)
        return cal_lenght

    def cmd_Tn(self, tool):
        self.toolhead.set_extruder(self.extruder[tool])
        self.gcode.extruder = self.extruder[tool]
        self.gcode.reset_last_position()
        self.gcode.extrude_factor = 1.
        self.gcode.base_position[3] = self.gcode.last_position[3]
        self.gcode.active_extruder = tool
        
    def filament_sensoring_callback(self,eventtime):
        if self.state == ('ready'):
            self.read_data()
            #self.filament_sensoring_data['print_time'] = self.get_print_time()
        return eventtime + 0.1
    # store/restore
    def restore(self):
        cfg_name = ("FS_CALIBRATION")
        sect = self.config.get_prefix_sections(cfg_name)
        for s in sect:
            if s.get_name() == ('FS_CALIBRATION'):
               pat9125_mm_factors = s.get('pat9125_mm_factor')
               self.pat9125_mm_factor = pat9125_mm_factors.split(',')
               for q in range (len(self.pat9125_mm_factor)): 
                   self.pat9125_mm_factor[q] = float(self.pat9125_mm_factor[q])
               self.gcode.respond_info("FS: Restored %s" %(self.pat9125_mm_factor))
               break
        else:
           self.gcode.respond_info("FS: No calibration record found, using default %s" %(self.pat9125_mm_factor))       
                    
    def store(self):
        configfile = self.printer.lookup_object('configfile')
        cfg_name = ("FS_CALIBRATION")
        configfile.set(cfg_name, 'pat9125_mm_factor', str(self.pat9125_mm_factor).strip('[]'))
        self.gcode.respond_info("FS: Use #SAVE_CONFIG to store calibration")
                
    def stats(self, eventtime):
        #return False, 'Filament sensoring %s: '%(self.filament_sensoring_data)
        if self.state == ('ready'):
            active_extruder = self.gcode.get_active_extruder()
            if active_extruder is None:
                return False, 'FS: T:None'
            else:
                try:    
                    t = ('T%i'%active_extruder)
                    data = self.filament_sensoring_data[t]
                    if data['F']: finda = ('present')
                    else: finda = ('runout')
                    shutter = data['S']
                    brightness = data['B']
                    lenght = data['L']
                    lenght_m = (data['Lmm'] / 1000)
                    cal_l = self.pat9125_mm_factor[active_extruder]
                    move = data['M']
                    return False, 'FS: %s filament:%s lenght:%0.3f m cal_l:%0.6f move:%s'%(t,finda,lenght_m,cal_l,move)
                except KeyError:
                    return False, 'FS: FAIL'
                
def load_config(config):
    return Filament_sensoring(config)