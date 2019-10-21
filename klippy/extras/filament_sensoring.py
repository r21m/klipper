import homing, kinematics.extruder
import toolhead, gcode, heater, logging
#generic
import serial  #pip install pyserial
import os
#import numpy

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
        if not ('X') or not ('Y') in self.pat9125_use_axis:
            raise homing.EndstopError("Filament sensoring: Use X or Y axis!")
        self.calibration_lenght = self.config.getfloat('calibration_lenght', 100)
        self.calibration_feed = self.config.getint('calibration_feed', 120)
            #variables
        self.filament_sensoring_data = {}
        self.state = {'state':None,'clog':[False] * self.maxval_t,'runout':[False] * self.maxval_t}
        self.t_prev = [0] * self.maxval_t
        self.l_pre = [0] * self.maxval_t
        self.active_extruder = None
        self.filament_sensoring_callback_tick = 0.1
        self.fs_runout_enable = None
        self.fs_clogged_enable = None
        self.ratio_old = None
            #
        self.gcode.register_command('QUERY_FS', self.cmd_query_filament_sensoring,desc='')
        self.gcode.register_command('CAL_FS', self.cmd_calibrate_lenght,desc='')
        self.gcode.register_command('RESET_FS', self.cmd_reset,desc='')
        self.gcode.register_command('FS_RUNOUT_ENABLE', self.cmd_fs_runout_enable,desc = '')
        self.gcode.register_command('FS_CLOGGED_ENABLE', self.cmd_fs_clogged_enable, desc = '')
        self.filament_sensoring_timer = reactor.register_timer(self.filament_sensoring_callback)        
           #scripts
        self.gcode_before_cal = self.config.get('gcode_before_cal', '')
        self.gcode_after_cal = self.config.get('gcode_after_cal', '')
        self.gcode_if_runout = self.config.get('gcode_if_runout', '')
        self.gcode_if_clogged = self.config.get('gcode_if_clogged', '')
           #clogg detection
        self.no_extrusion_time = self.config.getint('no_extrusion_time', 1) #reset time
        self.no_extrusion_max_len = self.config.getint('no_extrusion_max_len', 3)
        self.no_extrusion_max_ratio = self.config.getfloat('no_extrusion_max_ratio', 1.3)
        self.max_ratio_change = self.config.getfloat('max_ratio_change', 10)
        self.ratio_array = [0.0] * 10
        self.in_process_cal = self.config.getboolean('in_process_cal', True)
        if self.in_process_cal:
           self.last_position_array = [0.0] * self.maxval_t
           #self.calibration_lenght_array = (0.1 * self.calibration_lenght,self.calibration_lenght,10.0 * self.calibration_lenght)
           #self.calibration_stage = [0] * self.maxval_t
           self.in_process_cal_stat = None
        
    def handle_ready(self):
        self.init_board()
        reactor = self.printer.get_reactor()
        reactor.update_timer(self.filament_sensoring_timer,reactor.NOW)
        self.toolhead = self.printer.lookup_object('toolhead')
        self.restore()
        self.gcode.respond_info("FS: Ready")
        self.state['state'] = ('ready')
        return

    def connect(self):
        try:
            self.sens_serial = serial.Serial(self.serial_port,self.serial_speed,timeout = self.serial_timeout)
        except serial.SerialException:
            raise homing.EndstopError('FS: Connection exception')
        
    def init_board(self):
        self.connect() 
        for q in range(self.maxval_t):
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
        read_data_stat = None
        try:
            self.send_cmd('E1') #read ascii
            s = self.sens_serial.readline()
        except serial.SerialException:
           #self.state = ('serial exception')
           self.sens_serial.close()
           self.connect()
           read_data_stat = ('SerialException')
            
        s = s[:-2:]
        s = s.split(',')

        out = []
        for val in s:
           try:
                out.append(int(val))
           except ValueError:
                read_data_stat = ('Fail')
                break
            
        val_dict = {}            
        if (len(out) == (6*self.pat9125_used)) and not (read_data_stat):
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
                    val_dict['M'] = (val_dict['L'] - self.l_pre[tool_num])
                    val_dict['P'] = self.get_extruder_position()
                    if val_dict['P'] is None:
                        val_dict['P'] = 0
                    try:    
                        val_dict['R'] = val_dict['P'] / val_dict['Lmm']
                    except ZeroDivisionError:
                        val_dict['R'] = 0.0
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
    def cmd_query_filament_sensoring(self,params):
        self.gcode.respond_info("FS: data:")
        self.gcode.respond_info(self.filament_sensoring_data)
        return
        
    def cmd_reset(self,params):
        if ('R') in params:
            num = int(params['R'])
            self.reset_pat(num)
            self.gcode.respond_info("FS: Resetting %i" %num)
        return
    
        #Calibrate
    def cmd_calibrate_lenght(self, params): #FS_CALIBRATE E<NUM>
        if self.in_process_cal:
            self.gcode.respond_error('FS: ERROR: "in_process_calibration" is active')
            return
        _len = self.calibration_lenght
        _feed = self.calibration_feed
        store = False
    
        if ('T') in params:
            e_num = params['T']
            self.cmd_Tn(e_num)
        else:    
            e_num = self.gcode.get_active_extruder() #current active
            if e_num is None:
                self.gcode.respond_error('FS: No active extruder')
                return     
        if ('E') in params: _len = int(params['E'])
        if ('F') in params: _feed = int(params['F'])
                              
        self.gcode.respond_info("FS: Calibration begin T:%i E:%i F:%i"%(e_num,_len,_feed))        
        self.gcode.run_script_from_command(self.gcode_before_cal)        
        self.wait_for_finish_scripts()        
        cal_lenght = self.cmd_calibrate(e_num, _len,_feed)
        if (cal_lenght > 1):
            self.gcode.respond_error('FS: Calibrated value is > 1')
            return
        delta_cal = (cal_lenght - self.pat9125_mm_factor[e_num])   
        self.gcode.respond_info("FS: Calibrated T:%i new:%f old:%f delta cal:%f"%(e_num,cal_lenght,self.pat9125_mm_factor[e_num],delta_cal))
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
        
    def cmd_fs_runout_enable(self,params):
        if 'E' in params:
            self.fs_runout_enable = True
        elif 'D' in params:
            self.fs_runout_enable = False        
        return
    def cmd_fs_clogged_enable(self,params):
        if 'E' in params:
            self.fs_clogged_enable = True
        elif 'D' in params:
            self.fs_clogged_enable = False     
        return
        #    
    def cmd_Tn(self, tool):
        self.toolhead.set_extruder(self.extruder[tool])
        self.gcode.extruder = self.extruder[tool]
        self.gcode.reset_last_position()
        self.gcode.extrude_factor = 1.
        self.gcode.base_position[3] = self.gcode.last_position[3]
        self.gcode.active_extruder = tool
        
        
    def filament_sensoring_callback(self,eventtime):
        if self.state['state'] == ('ready'):
            self.read_data()
            t = self.active_extruder
            if t is not None:
                t = ('T%i'%self.active_extruder)
                if self.in_process_cal and (t in self.filament_sensoring_data):
                    # jednou za > self.calibration_lenght mm drahy
                    data = self.filament_sensoring_data[t]                
                    self.in_process_cal_stat = ('check')
                    actual_position = data['P']
                    measured_position = data['Lmm']                
                    delta = actual_position - self.last_position_array[self.active_extruder]
                    #logging.info('autocal: check cnd delta: %f actual_position:%f measured_position: %f'%(delta,actual_position,measured_position))
                    if (delta >= self.calibration_lenght) and (measured_position > 0):
                        self.in_process_cal_stat = ('run')
                        self.last_position_array[self.active_extruder] = actual_position
                        ratio = self.pat9125_mm_factor[self.active_extruder] * ( actual_position / measured_position )
                        new_pos = data['L'] * ratio                        
                        self.pat9125_mm_factor[self.active_extruder] = ratio
                        logging.info('>>>autocal: tool:%i new_ratio:%f delta: %f actual:%f measured:%f recalc:%f'%(self.active_extruder,ratio,delta,actual_position,measured_position,new_pos))
                    self.in_process_cal_stat = ('done')
                if (t in self.filament_sensoring_data):# and self.fs_clogged_enable:
                    data = self.filament_sensoring_data[t]
                    if self.ratio_old is None:
                        self.ratio_old = data['R']
                    if data['R']:
                        super_ratio = self.ratio_old / data['R']
                        delta = abs(1 - super_ratio)
                        if delta > 0.004:
                            logging.info('>>>clogged super_ratio:%0.6f delta:%0.6f'%(super_ratio,delta))
                        elif delta > 0.0004:    
                            logging.info('>>>clogged warning super_ratio:%0.6f delta:%0.6f'%(super_ratio,delta))    
                        self.ratio_old = data['R']
                                    
        return eventtime + self.filament_sensoring_callback_tick
         
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
        if self.state['state'] == ('ready'):
            self.active_extruder = self.gcode.get_active_extruder() 
            if self.active_extruder is None:
                return False, 'FS: T:None'
            else:
                try:
                    pos = self.get_extruder_position() / 1000               
                    t = ('T%i'%self.active_extruder)
                    data = self.filament_sensoring_data[t]
                    if data['F']: finda = ('present')
                    else: finda = ('runout')
                    shutter = data['S']
                    brightness = data['B']
                    lenght = data['L']
                    lenght_m = (data['Lmm'] / 1000)
                    cal_l = self.pat9125_mm_factor[self.active_extruder]
                    move = data['M']
                    ratio = data['R']
                    return False, 'FS: %s filament:%s lenght:%0.3f m cal_l:%0.6f move:%i pos:%f ratio:%s'%(t,finda,lenght_m,cal_l,move,pos,ratio)
                except KeyError:
                    return False, 'FS: FAIL'
                    
    def get_extruder_position(self):
        extruder_pos = self.toolhead.get_position()[3]
        logging.info('get_extruder_position',extruder_pos)
        return extruder_pos
        
    def get_stat_extruder_position(self):
        pos = self.gcode.get_stat_extruder_position(self.active_extruder)
        logging.info('get_stat_extruder_position', pos)
        return pos
        
    def get_extruder_abs_sum_position(self):
        pos = self.gcode.get_stat_extruder_abs_sum_position(self.active_extruder)
        logging.info('get_extruder_abs_sum_position', pos)
        return pos
        
def load_config(config):
    return Filament_sensoring(config)