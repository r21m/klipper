#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

#Klipper
import homing, kinematics.extruder
import toolhead, gcode, heater, logging
#generic
import serial  #pip install pyserial
import os
import pickle
import time
class error(Exception):
    pass

####################
#MMU2 control class#
####################

class MMU2control:
    error = error
    def __init__(self,config):
            #objects
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        self.dispay = None
        self.heater = None
        self.gcode_id = None
            #parameters
        self.tool_per_unit = self.config.getint('tool_per_unit', minval=1, maxval = 8, default=5)
        self.enable = self.config.getboolean('enable', False)
            #unit parameters
        ports = config.get('serial_ports')
        self.serial_ports = ports.split(',')
        self.serial_speed = self.config.getint('serial_speed', 115200)
        self.serial_timeout = self.config.getint('serial_timeout', 0.05)
        self.unit_count = len(self.serial_ports)
           #timeout
        self.change_timeout = config.getint('change_timeout',120)
           #assign extruder unit
        assign_extruders = self.config.get('assign_extruders')
        self.assign_extruder = assign_extruders.split(',')
        self.initial_reset = self.config.getboolean('initial_reset', True)
           #PAT9125
        self.use_pat9125 = self.config.getboolean('use_pat9125', False)
        if self.use_pat9125:
            self.pat9125_mm_factor = self.config.getfloat('pat9125_mm_factor', minval=0.001, maxval = 1.0, default=0.0307125)
            self.pat9125_use_axis = self.config.get('pat9125_use_axis','Y')
            #check temp
        self.min_temp = self.config.getint('min_temp', minval=190, default=200)
        self.min_temp = self.min_temp - 5
            #change mode
        self.change_mode = self.config.getint('change_mode',minval = 0, maxval = 20, default=0)
        self.continue_loading_count = self.config.getint('continue_loading_count',minval = 0, maxval = 3, default=1)
            #HW RESET
        self.hw_reset = [None for x in range(self.unit_count)]
        reset_pins = self.config.get('reset_pins')
        self.reset_pins = reset_pins.split(',')
        ppins = self.printer.lookup_object('pins')
        self.last_value_time = 0.0
        self.hw_reset_pulse_time = 0.25
        for q in range(self.unit_count):
            self.hw_reset[q] = ppins.setup_pin('digital_out', self.reset_pins[q])
            self.hw_reset[q].setup_max_duration(0.0)
            self.hw_reset[q].setup_start_value(True, True)
        #variables#
        self.state = None
        self.maxval_t = None
        self.active_tool = None
        if self.unit_count == 0: self.maxval_t = self.tool_per_unit
        else: self.maxval_t = (self.unit_count * self.tool_per_unit)-1
        self.unit = [None for x in range(self.unit_count)]
        self.extruder = [None for x in range(self.unit_count)]
        #MEM
        self.mmu2mem = mmu2_memory()
        self.init_scripts()
        self.init_commands()


    def printer_state(self, state):
        if (state == 'ready'):
            msg = ('MMU2 control: not ready')
            self.state = ('not ready')
            logging.info(msg)
            self.gcode.respond_info(msg)
            self.init_display()
            self.toolhead = self.printer.lookup_object('toolhead')
            self.heater = self.printer.lookup_object('heater')

            for q in range (self.unit_count):
                self.extruder[q] = self.printer.lookup_object(self.assign_extruder[q])

            self.gcode_id = self.heater.heaters_gcode_id
            if self.initial_reset:
                self.reset_units()

            self.init_units()
            self.connect_all()

            self.restore()
            self.reactor.register_timer(self.mmu2_callback,self.reactor.NOW)
            self.reactor.register_timer(self.mmu2_callback_runout,self.reactor.NOW)
            msg = ('MMU2 control: ready')
            logging.info(msg)
            self.gcode.respond_info(msg)
            self.state = ('ready')

    def init_units(self):
        '''
        en: create mmu2 object list

        cz: vytvori seznam instanci MMU2
        '''
        logging.info('MMU2 control add:')
        for q in range(self.unit_count):
            _unit_index = q
            logging.info('\tunit%s'%(_unit_index))

            self.unit[q] = mmu2(config = self.config,
            serial_port = self.serial_ports[q],
            serial_speed = self.serial_speed,
            serial_timeout = self.serial_timeout,
            unit_index = _unit_index,
            change_timeout = self.change_timeout)

    def init_scripts(self):
        #script#
        self.load_unload_profiles = self.config.getint('load_unload_profiles', minval = 1, maxval = (self.tool_per_unit*self.unit_count), default=1)
        self.gcode_before_M6 = self.config.get('gcode_before_m6', '')
        self.gcode_after_M6 = self.config.get('gcode_after_m6', '')
        self.gcode_if_fail_M6 = self.config.get('gcode_if_fail_m6', '')
        self.gcode_if_runout = self.config.get('gcode_if_runout', '')
        #list script

        # lu = [0,0,0,0,0,0,0,0,0]
        #
        self.lu_mem = []
        self.gcode_load_profile = []
        self.gcode_unload_profile = []
        for num in range (self.load_unload_profiles):
            self.gcode_load_profile.append(self.config.get(('gcode_load_profile%i' %(num)), ''))
            self.gcode_unload_profile.append(self.config.get(('gcode_unload_profile%i' %(num)), ''))

        for tool in range (self.unit_count*self.tool_per_unit):
            self.lu_mem.append(0)

        self.gcode_if_tool_after_M6 = []
        self.gcode_if_tool_before_M6 = []
        for tool in range (self.unit_count*self.tool_per_unit):
            self.gcode_if_tool_before_M6.append(self.config.get(('gcode_if_tool%i_before_m6' %(tool)), ''))
            self.gcode_if_tool_after_M6.append(self.config.get(('gcode_if_tool%i_after_m6' %(tool)), ''))

        self.gcode_if_unit_before_M6 = []
        self.gcode_if_unit_after_M6 = []
        for unit in range (self.unit_count):
            self.gcode_if_unit_before_M6.append(self.config.get(('gcode_if_unit%i_before_m6'%(unit)), ''))
            self.gcode_if_unit_after_M6.append(self.config.get(('gcode_if_unit%i_after_m6'%(unit)), ''))

    def init_commands(self):
        #desc
        self.desc_M403 = 'M403\n Send the filament type to the MMU2.0 unit.\nE<extruder number>, F<filament type>\n(0: default; 1:flex; 2: PVA)'
        self.desc_M701 = 'M701\n E<num> load filament \n EJECT_EXTRUDER UNIT=unit0 E=0'
        self.desc_M702 = 'M702\n U<num> unload \nC unload from nozzle \nL<num> load filament to MMU'
        self.desc_M6 = ''
        self.desc_QUERY_MMU2 = ''
        self.desc_SET_MMU2 = ''
        self.desc_RESET_MMU2 = ''
        #commands
        self.gcode.register_command('M403',self.cmd_M403,desc=self.desc_M403)
        self.gcode.register_command('M701',self.cmd_M701,desc=self.desc_M701)
        self.gcode.register_command('M702',self.cmd_M702,desc=self.desc_M702)
        self.gcode.register_command('M6',self.cmd_M6,desc=self.desc_M6)
        self.gcode.register_command('QUERY_MMU2',self.cmd_QUERY_MMU2,desc=self.desc_QUERY_MMU2)
        self.gcode.register_command('SET_MMU2',self.cmd_SET_MMU2,desc=self.desc_SET_MMU2)
        self.gcode.register_command('RESET_MMU2',self.cmd_RESET_MMU2,desc=self.desc_RESET_MMU2)
        self.gcode.register_command('TEST0',self.cmd_TEST0,desc=None)
        self.gcode.register_command('TEST1',self.cmd_TEST0,desc=None)

    def init_display(self):
        printer_objects = self.printer.objects
        if ('display') in printer_objects: self.display = self.printer.lookup_object('display')
        else: self.display = None

#commands
    def cmd_M6(self,params):
        '''


        cz:
        Provadi vymenu filamentu:
        M6 T<num> vymeni/zavede filament <num>
        M6 L zavede filament do extruderu
        M6 U vytahne filament z extruderu
        '''

        self.gcode.respond_info('MMU2 control: M6: prepare')
        self.set_message('Prepare change')

        index = None
        T_param = None
        T_param_unit = None

        act_unit_index,act_t_unit = self.get_correct_values(self.active_tool)

        if ('T') in params:

            T_param = self.gcode.get_int('T', params, minval=0, maxval=self.maxval_t)
            index,T_param_unit = self.get_correct_values(T_param)
            finda = self.unit[index].state['finda']
            #self.gcode.respond_info('FINDA %s'%finda)


            if params['T'] == (None or ('')):
                self.gcode.respond_info('MMU2 control M6: unload')
                self.set_message('Unloading filament')
                self.unload_filament_from_extruder(act_unit_index,T_param)
                self.unit[act_unit_index].unload_filament()
                self.active_tool = None
                self.store()
                return

            if T_param == self.active_tool:
                self.gcode.respond_info('MMU2 control M6: T<old> = T<new> no change')
                self.set_message('T<old>=T<new>')
                return

            if (self.unit[index].state['active_tool_in_unit'] == T_param_unit) and (finda):
                self.gcode.run_script_from_command(self.gcode_if_unit_before_M6[index])
                self.gcode.respond_info('MMU2 control M6: T<old> = T<new> no change, unit change')
                self.set_message('T<old>=T<new> unit change')
                self.call_abort(index)
                self.load_filament_to_extruder(index,T_param)
                self.active_tool = T_param_unit
                self.store()
                self.gcode.run_script_from_command(self.gcode_if_unit_after_M6[index])
                return

        if not (self.enable):
            self.gcode.respond_info('MMU2 control M6: not enabled, use <SET_MMU A> to enable')
            self.set_message('M6 not enabled')
            return

        if self.active_tool == None: T_old ='?'
        else: T_old = self.active_tool
        self.gcode.respond_info('MMU2 control: M6: Tool change T%s>T%s'%(T_old,T_param))
        self.set_message('Change T:%s>T:%s'%(T_old,T_param))

        self.gcode.run_script_from_command(self.gcode_if_unit_before_M6[index])

        if (finda == (True or None)):
            self.unload_filament_from_extruder(index,T_param)

        #change begin
        self.gcode.run_script_from_command(self.gcode_before_M6)
        self.gcode.run_script_from_command(self.gcode_if_tool_before_M6[T_param])
        change_state = self.call_filament_change(T_param_unit,index)

        if (change_state == False):
            self.gcode.respond_error('MMU2 control M6: Tool change FAIL, unit:%i tool:%i' %(index,T_param_unit))
            self.set_message('Tool change T% FAIL'%T_param_unit)
            if not (self.gcode_if_fail_M6 == ''):
                self.active_tool = None
                self.gcode.run_script_from_command(self.gcode_if_fail_M6)
                return
            else:
                self.active_tool = None
                self.store()
                self.set_message('Change FAIL')
                raise error('MMU2 control M6: Tool change FAIL, unit:%i tool:%i' %(index,T_param_unit))

        #loading to extruder
        for q in range(self.continue_loading_count):
           self.call_continue_loading(index)
        else:
           self.call_abort(index)
           self.reactor_pause(1)

        self.load_filament_to_extruder(index,T_param)
        self.reactor_pause(1)
        #Set variables
        self.active_tool = T_param
        #store
        self.store()

        self.gcode.run_script_from_command(self.gcode_if_unit_after_M6[index])
        self.gcode.run_script_from_command(self.gcode_after_M6)
        #info
        self.gcode.respond_info('MMU2 control: Tool change finished, active T:%s'%(self.active_tool))
        self.set_message('Active T:%s'%(self.active_tool))
        return

    def cmd_M403(self,params):
        '''en: Send the filament type to the MMU2.0 unit.
        E<extruder number>, F<filament type>(0: default; 1:flex; 2: PVA)

        cz: Nastavuje typ filamentu
        E extruder, F<typ filamentu>(0: default; 1:flex; 2: PVA)

        M403 E<num> F<num>
        '''
        if ('E') in params and ('F') in params:
            _e = self.gcode.get_int('E', params, minval=0, maxval=self.maxval_t)
            f = self.gcode.get_int('F', params, minval=0, maxval=2)
            unit_index,e = self.get_correct_values(_e)
            self.unit[unit_index].set_filament_type(e,f)
            return

    def cmd_M701(self, params):
        '''
        load filamen to MMU
        M701 E<num>
        '''
        if ('E') in params:
            e = self.gcode.get_int('E', params, minval=0, maxval=self.maxval_t)
            unit_index,e = self.get_correct_values(e)
            if self.unit[index].state['finda']:
                self.gcode.respond_error('MMU2 control: M701: unit%s filament detected!'%(unit_index))
                return
            self.unit[unit_index].eject_extruder()
            return

    def cmd_M702(self, params):
        '''
        U<num> unload
        C unload from nozzle
        L<num> load filament to MMU
        '''
        is_C = is_U = is_L = False
        cnum = unum = lnum = None

        is_C,cnum = self.get_int_value_from_param(params,'C')
        is_U,unum = self.get_int_value_from_param(params,'U')
        is_L,lnum = self.get_int_value_from_param(params,'L')

        T_active = self.active_tool
        act_unit_index = None
        act_t_unit = None
        act_unit_index,act_t_unit = self.get_correct_values(T_active)

        if not (bool(is_C) ^ bool(is_U) ^ bool(is_L)):
            self.gcode.respond_info('MMU2 control: M702 Wrong parameter')
            return

        if (is_U):
            index,T_param_unit = self.get_correct_values(unum)
            if self.unit[index].state['finda']:
                self.gcode.respond_info('MMU2 control: M702 unit%s filament detected!' %(index))
                return
            self.unload_filament_from_extruder(index,T_param_unit)
            self.call_filament_unload(T_param_unit,index)
            self.active_tool = None
            self.store()


        if (is_L and lnum != None):
            index,T_param_unit = self.get_correct_values(lnum)
            self.call_load_filament(T_param_unit,index)


        if (is_C):# unload current active      #U0
            index,T_param_unit = self.get_correct_values(unum)
            self.unload_filament_from_extruder(index,T_param_unit)
            if (act_unit_index == None) and (cnum == None):
                for q in range(self.unit_count):self.call_filament_unload(0,q)
                self.active_tool = None

            elif (act_unit_index == None) and not (cnum == None):
                self.call_filament_unload(0,cnum)

            elif  (act_unit_index >= 0) and (cnum == None):
                self.call_filament_unload(0,act_unit_index)

        if self.unit[act_unit_index].state['cmd_state'] ==('fail'):
            raise error('MMU control: M702 error')

    def cmd_SET_MMU2(self,params):

        if ('T' in params): #set active tool
            if params['T'] == ('') or None:
                #kdyz je T<> nastavit None
                for q in range(self.unit_count):self.unit[q].state['active_tool_in_unit'] = None
                self.active_tool = None
            else:
                T_param = self.gcode.get_int('T', params, minval=0, maxval=self.maxval_t)
                index,T_param_unit = self.get_correct_values(T_param)

                self.unit[index].state['active_tool_in_unit'] = T_param_unit
                self.active_tool = T_param

            self.gcode.respond_info('MMU2 control SET_MMU2: active T: %s'%(self.active_tool))

        elif ('A') in params:
           self.enable = True
           self.gcode.respond_info('MMU2 control SET_MMU2: M6 enabled')
           self.set_message('M6 enabled')

        elif ('D') in params:
            self.enable = False
            self.gcode.respond_info('MMU2 control SET_MMU2: M6 disabled')
            self.set_message('M6 disabled')

        elif ('P') in params:
            '''
            #MMU2_SET P1 T0
            '''
            T_param = self.gcode.get_int('T', params, minval=0, maxval=self.maxval_t)
            P_param = self.gcode.get_int('P', params, minval=0, maxval=(self.unit_count*self.tool_per_unit))
            self.lu_mem[T_param] = P_param
            self.gcode.respond_info('MMU2 control SET_MMU2: set LU profile:% tool:%'%(P_param,T_param))

        elif ('S') in params:
            self.gcode.respond_info('MMU2 control SET_MMU2: store')
            self.store()

    def cmd_QUERY_MMU2(self,params):
        if self.enable: m6_stat = ('enabled')
        else: m6_stat = ('disabled')

        self.gcode.respond_info('QUERY_MMU2:\n')
        self.gcode.respond_info('\tM6: %s'%(m6_stat))
        tool = ('None')
        if self.active_tool != None: tool = str(self.active_tool)

        self.gcode.respond_info(('\tActive tool: %s')%(tool))
        self.gcode.respond_info(('\tL/U memory:%s')%(self.lu_mem))

        for q in range(self.unit_count):
            if self.unit[q] == None:
                self.gcode.respond_info('\tUnit %s None'%q)
                return
            self.unit[q].get_finda_state()
            self.gcode.respond_info(('\tunit:%s state: %s\n')%(q,self.unit[q].state))



    def cmd_RESET_MMU2(self,params):
        if ('U') in params:
            self.reset_unit(params['U'])
            return
        else:
            self.reset_units()
            return

    def cmd_TEST0(self,params):
        '''
        na volne testovani
        '''
        self.gcode.respond_info('<<<TEST0>>>')
        logging.info('<<<TEST0>>>')
        #
        ext = kinematics.extruder.get_printer_extruders(self.printer)
        self.gcode.respond_info(str(ext))
        #
        logging.info('<<<TEST0>>>')
        self.gcode.respond_info('<<<TEST0>>>')
        return

    def cmd_TEST1(self,params):
        '''
        na volne testovani
        '''
        self.gcode.respond_info('<<<TEST1>>>')
        logging.info('<<<TEST1>>>')
        #

        #
        logging.info('<<<TEST1>>>')
        self.gcode.respond_info('<<<TEST1>>>')
        return

####
    def get_correct_values(self,E_par):
        '''

        T<5>     = index =1,E=0
        T<0>     = index =0,E=0
        T<9>     = index =1,E=4
        T<None>  = index = None, E = None
        '''

        if E_par == None: return None,None

        index = 0
        for q in range (self.unit_count):
            if E_par < self.tool_per_unit:
                break
            E_par = E_par - (self.tool_per_unit)
            index = index + 1
        return index,E_par
####
    def get_int_value_from_param(self,_params,key):
        is_key = False
        num = None
        if (key) in (_params):
            is_key = True
            try:
                num = _params[key]
                if len(num) == 0:
                    num = None
                else:
                    num = int(num)
            except ValueError:
                self.gcode.respond_error('MMU2 control: error in parameter: %s key: %s' %(_params,key))
                return
        return is_key,num
#--------------------------------------------------

    def check_move_extruder(self,unit_num):
       '''
       testuje teplotu extruderu pro vymenu
       '''
       if unit_num == None: return None
       extruder = self.assign_extruder[unit_num]
       _eventtime = self.reactor.monotonic()

       for _key, _val in self.gcode_id.items():
           if _val == extruder:
               temp = self.heater.get_heater_by_gcode_id(_key).get_temp(_eventtime)
               break

       temp = round(temp[0],0) # merena teplota

       if temp < self.min_temp:
           self.gcode.respond_error('MMU2 control: extruder mintemp')
           self.set_message('Extruder mintemp')
           raise error('MMU2 control: extruder mintemp:%s measured:%s'%(self.min_temp,temp))
           return False
       else:
           return True
#call
    def call_filament_change(self,tool,unit_index):
        self.extruder_motor_off(unit_index)
        self.unit[unit_index].filament_change(tool)

        if self.unit[unit_index].state['cmd_state'] == ('ready'): return True
        else: return False

    def call_load_filament(self,tool,unit_index):
        self.unit[unit_index].load_filament(tool)

    def call_continue_loading(self,unit_index):
        self.gcode.respond_info('MMU2 control: Continue loading...')
        self.unit[unit_index].continue_loading()

    def call_filament_unload(self,tool,unit_index):
        self.unit[unit_index].unload_filament(tool)

    def call_abort(self,unit_index):
        self.unit[unit_index].abort()

#load/unload to extruder
    def load_filament_to_extruder(self,index = 0,tool = 0):
        #self.gcode.respond_info('Load filament')
        self.check_move_extruder(index)
        try:
            profile_num = self.lu_mem[tool]
            self.gcode.run_script_from_command(self.gcode_load_profile[profile_num])
        except IndexError:
            self.gcode.run_script_from_command(self.gcode_load_profile[0])
        self.wait_for_finish_scripts()

    def unload_filament_from_extruder(self,index = 0,tool = 0):
        #self.gcode.respond_info('Unload filament')
        self.wait_for_finish_scripts()
        self.check_move_extruder(index)
        try:
            profile_num = self.lu_mem[tool]
            self.gcode.run_script_from_command(self.gcode_unload_profile[profile_num])
        except IndexError:
            self.gcode.run_script_from_command(self.gcode_unload_profile[0])
        self.wait_for_finish_scripts()
        self.extruder_motor_off(index)

    def wait_for_finish_scripts(self):
        self.toolhead.wait_moves()

    def reset_unit(self,unit_num):
        logging.info('\tunit:%s'%unit_num)
        PIN_MIN_TIME = 0.1
        print_time = self.get_print_time()
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)

        self.hw_reset[unit_num].set_digital(print_time, 0)
        self.reactor_pause(self.hw_reset_pulse_time)

        print_time = self.get_print_time()
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)

        self.hw_reset[unit_num].set_digital(print_time, 1)
        self.reactor_pause(self.hw_reset_pulse_time)

        self.last_value_time = print_time
        self.reactor_pause(10)

    def reset_units(self):
        logging.info('MMU2 control: reset:')
        for q in range(self.unit_count): self.reset_unit(q)

    def disconnect_all(self):
        for q in range(self.unit_count):self.unit[q].disconnect()

    def connect_all(self):
        msg = ("MMU2 control: connecting units:")
        logging.info(msg)
        for q in range(self.unit_count):
            for k in range(0,12,1):
                cn_state = self.unit[q].connect()
                if cn_state:
                    msg = ('\tunit:%s connected' % (q))
                    logging.info(msg)
                    break
                else:
                    self.reactor_pause(5)
            if not cn_state:
                msg = ('\tport%s connect fail' % (self.serial_ports[q]))
                self.gcode.respond_error(msg)
                raise error(msg)

    def restore(self):
        mem = self.mmu2mem.read()
        logging.info('MMU2 control restore')
        try:
            self.active_tool = mem['active_tool']
        except KeyError:
            self.active_tool = None
        logging.info('\tactive tool:%s'%(self.active_tool))

        _active_tool_in_unit = mem['active_tool_in_unit']

        if self.active_tool != None: self.set_Tn(self.active_tool)

        for q in range(self.unit_count):
           tool = ('None')
           if _active_tool_in_unit[q] != None: tool = str(_active_tool_in_unit[q])

           self.unit[q].state['active_tool'] = _active_tool_in_unit[q]
           logging.info('\tactive tool:%s in unit:%s'%(tool,q))

        try:
            self.lu_mem = mem['lu_mem']
        except KeyError:
            pass

    def store(self):
        _active_tool_in_unit = [None for x in range(self.unit_count)]

        for q in range(self.unit_count):
            _active_tool_in_unit[q] = self.unit[q].state['active_tool_in_unit']
        mem = {'active_tool': self.active_tool,'active_tool_in_unit' : _active_tool_in_unit,'lu_mem': self.lu_mem}
        logging.info('MMU2 control store:%s'%(mem))
        self.mmu2mem.write(mem)

    def reset_store(self):
        for q in range(self.unit_count):
            self.unit[q].state['active_tool_in_unit'] = None
        self.active_tool = None

        _active_tool_in_unit = [None for x in range(self.unit_count)]

        mem = {'active_tool': self.active_tool,'active_tool_in_unit' : _active_tool_in_unit}
        logging.info('MMU2 control store: RESET')
        self.mmu2mem.write(mem)

    def set_Tn(self,Tn):
        #self.gcode.cmd_Tn(Tn)
        self.gcode.respond('T:%s'%(Tn))

    def get_print_time(self):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        return print_time

    def reactor_pause(self,time):
        _eventtime = self.reactor.monotonic()
        _eventtime = self.reactor.pause(_eventtime + time)
    #LCD
    def set_message(self,msg,msg_time = None):
        if self.display != None:
            self.display.set_message(msg,msg_time)
    #stepper
    def extruder_motor_off(self,index):
        time = self.get_print_time()
        self.extruder[index].stepper.motor_enable(time, 0)
        self.extruder[index].need_motor_enable = True
        self.reactor_pause(0.1)

    #callback, stats
    def mmu2_callback(self,eventtime):
        return eventtime + 2

    def mmu2_callback_runout(self,eventtime):
        return eventtime + 5

    def stats(self, eventtime):
        state_list = []
        for q in range(self.unit_count):
            if self.unit[q] != None:
                state_list.append (self.unit[q].state)
            else:
                state_list.append(None)

        return False, '%s: tool:%s unit state:%s lu_mem:%s' % ('MMU2 control',self.active_tool,state_list,self.lu_mem)

########################################################################################################################
#MMU2 class                                                                                                            #
########################################################################################################################

class mmu2():
    error = error
    def __init__(self,config,serial_port,serial_speed,serial_timeout,unit_index,change_timeout):
        '''
        vytvari objekt unit[q].mmu.
        '''
        #printer objects...
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.mmu2control = self.printer.lookup_object('mmu2control')
        #unit parameters
        self.serial_port = serial_port
        self.serial_speed = serial_speed
        self.serial_timeout = serial_timeout
        self.change_timeout = change_timeout
        self.unit_index = unit_index
        self.change_mode = self.mmu2control.change_mode
        #variables
        self.current_stepper_mode = None
        self.buildnr = None
        self.version = None
        self.drive_error = None
        self.mmu2_serial = None
        self.filament_type = []
        self.tool_per_unit = self.mmu2control.tool_per_unit
        for q in range(self.tool_per_unit):
            self.filament_type.append(0)
        #special, pat9125 filament sensor
        self.use_pat9125 = self.mmu2control.use_pat9125
        #internal state dict
        self.state = {'unit_index': self.unit_index,
                      'connect':None,
                      'cmd_state': None,
                      'finda':None,
                      'active_tool_in_unit':None,
                      'print_time':None,
                      'filament_type':self.filament_type}
        if self.use_pat9125:
            data = {'pat9125':None}
            self.state.update(data)
            self.pat9125_x = None
            self.pat9125_y = None
            self.pat9125_b = None
            self.pat9125_s = None
            self.pat9125_x_mm = None
            self.pat9125_y_mm = None
        self.handle_data_dict = {}
        #finda callback
        self.reactor.register_timer(self.callback_finda,self.reactor.NOW)
        #mem file
        self.mmu2mem = self.mmu2control.mmu2mem

#=======================
    def handle_data(self,data_write = '',timeout = 0):
        out = {'write': None,'tsw':0,'read':None,'data':[None],'tsr':0,'dt': 0}
        #self.gcode.respond_info('>>>data_write %s'%(data_write))
        #self.gcode.respond_info('>>>timeout %s'%(timeout))

        if timeout == None: timeout = 0
        self.serial_flush()

        self.state['cmd_state'] = ('busy')
        out['tsw'] = self.get_print_time()

        #self.gcode.respond_info('>>>write ')
        try:
            self.state['cmd_state'] = ('writing')
            self.mmu2_serial.write(data_write + '\n')
            out['write'] = data_write

        except serial.SerialException:
            self.state['cmd_state'] = ('serial_exception')
            return out
        #self.gcode.respond_info('>>>wait read')

        if timeout > 0:
            self.state['cmd_state'] = ('reading')
            try:
               self.mmu2_serial.timeout = timeout
               read = self.mmu2_serial.readline()
               if read:
                   out['data'],out['read'] = self.parse_read(read)
                   self.state['cmd_state'] = ('done')
               else:
                   self.state['cmd_state'] = ('timeout')
                   self.gcode.respond_info(('command: %s timeout')%(data_write))
            except serial.SerialException:
                self.state['cmd_state'] = ('serial_exception')
        else:
            self.state['cmd_state'] = ('done')

        out["tsr"] = self.get_print_time()
        #time DELTA
        try:
            out['dt'] =  (out['tsr'] - out['tsw'])
        except TypeError:
            out['dt'] = None

        if self.state['cmd_state'] == ('done'): self.state['cmd_state'] = ('ready')
        else: self.state['cmd_state'] == ('fail')
        #self.state['handle_data'] = out
        return out

#------parse read 123ok\n > 'data':123 'read':'ok'-------
    def parse_read(self,read_data,ok = ('ok')):
        '''
        ok\n =>>> 'read':'ok','data':[None]
        123ok =>> 'read':'ok','data':[123]
        3.14,123ok =>> 'read':'ok','data':[3.14,123]
        '''
        #self.gcode.respond('>>>read_data:%s'%(read_data))
        val = None
        val_list = []
        if (read_data == None) or (read_data == ''):
            val_list = [None]
            return val_list,None
        elif not ok in read_data:
            val_list = [None]
            return val_list,None
        else:          # oddelit data od 'ok'
            k = 0
            for q in read_data:#data od zacatku do 'o'
                if q == ok[0]:
                    val = read_data[0:k]
                    break
                k += 1
        if val is '':
            val_list = [None]
            return val_list,ok
        if (',') in val: #pro implementaci PAT9125 , vice hodnot
            if (',') in val[-1]:
                val = val[0:-1]
            val = val.split(',')
            for q in val:
                val_list.append(self.set_type(q))
            return val_list,ok
        else:
            val_list = [self.set_type(val)]
            return val_list,ok

    def set_type(self,data):
        try:
           if (".") in data: data = (float(data))
           else: data = (int(data))
        except ValueError:
            data = (str(data))
            if data == '': data = None
        return data

#commands
    def continue_loading(self):
        self.handle_data(data_write = 'C0')

    def unload_filament(self,u = 0):
        self.handle_data(data_write = ('U%i' %u),timeout = 60)

    def recover_after_eject(self,params):#???
        self.handle_data_dict = self.handle_data(data_write = ('R0'))

    def filament_change(self,t):
        mode = self.change_mode
        time = 2
        if mode == 0: #old mode, MK3
            self.handle_data_dict = self.handle_data(('T%i' %t), timeout = self.change_timeout)
        if mode == 1: #new mode with sensor, MK3S
            raise error('Not implemented')
        elif mode == 2: # time controlled change, experimental
            #self.change_timeout
            self.unload_filament()
            if self.state['cmd_state'] == ('fail'):
                raise error('MMU control: unit%s unload fail' %(self.unit_index))
            self.handle_data(('T%i' %t), timeout = 0.1)
            self.reactor_pause(time)
            self.abort()
            if not self.get_finda_state() or (self.state['cmd_state'] == ('fail')):
                raise error('MMU control: unit%s load fail' %(self.unit_index))

        elif mode == 3: # with PAT9125 lenght measuring, experimental, need more special variables
           raise error('Not implemented')
        elif mode == 10:
           raise error('Not implemented')

        self.state['active_tool_in_unit'] = t
        return self.handle_data_dict

    def load_filament(self,l = 0):
        self.handle_data(data_write = ('L%i' %l),timeout = self.change_timeout)


    def eject_extruder(self,e = 0):
        self.handle_data(data_write = ('E%i' %e),timeout = 50)


    def eject_filament(self,f = 0):
        self.handle_data(data_write = ('F%i' %(f)),timeout = 50)

    def abort(self):
        self.handle_data(data_write = ('A'),timeout = 0)

    def software_reset(self):
        self.handle_data(data_write = ('X0'),timeout = 50)
        self.disconnect()

    def direct_command(self,_cmd,_timeout = 1):
        if not _cmd:
            return
        retdict = self.handle_data(data_write = (_cmd),timeout = _timeout)
        return retdict

#finda, version, buildNr drive....
    def read_finda_state(self):
        self.handle_data_dict = self.handle_data(data_write = ('P0'),timeout = 50)
        finda = self.handle_data_dict['data'][0]
        if finda == None:
            self.state['finda'] = None
            return self.state['finda']
        else:
            self.state['finda'] = bool(finda)
            return self.state['finda']

    def read_version(self):
        self.handle_data_dict = self.handle_data(data_write = ('S1'),timeout = 50)
        #print(self.handle_data_dict)
        self.version = self.handle_data_dict['data'][0]
        return self.version

    def read_buildnr(self):
        self.handle_data_dict = self.handle_data(data_write = ('S2'),timeout = 50)
        self.bulidnr = self.handle_data_dict['data'][0]
        return self.bulidnr

    def read_drive_error(self):
        self.handle_data_dict = self.handle_data(data_write = ('S3'),timeout = 50)
        #print(self.handle_data_dict)
        self.bulidnr = self.handle_data_dict['data'][0]
        return self.drive_error

    def wait_for_user_cliq(self): #???
        self.handle_data_dict = self.handle_data(data_write = ('W0'), timeout = 120)

    def set_stepper_mode(self,mode = 0):
        self.handle_data(data_write = ('M%i' % mode),timeout = 0)

    def set_filament_type(self,e = 0,f = 0):
        self.handle_data(data_write = ('F%i E%i' %(f,e)),timeout = 0.5)
        self.filament_type[e] = f

    def restore_filament_type(self):
        for tool in range(self.tool_per_unit):
            self.handle_data(data_write = ('F%i E%i' %(tool,self.filament_type[tool])),timeout = 0.5)

    def get_filament_sensor_data(self):
        #K1 update and read all data, X,Y,B,S,F
        retdict = self.handle_data(data_write = ('K1'), timeout = 0.5)
        data = retdict['data']
        return data

    def reset_filament_sensor(self):
        self.handle_data(data_write = ('K3'),timeout = 1)

    def get_finda_state(self):
        state = self.read_finda_state()
        self.state['finda'] = state
        return state

    def get_cmd_state(self):
        return self.state['cmd_state']

    def get_mmu2_connect_state(self):
        return self.state['connect']

    def get_active_tool(self):
        return self.state['active_tool_in_unit']
#init
    def initial_read_and_set(self):
        self.read_buildnr()
        self.read_version()
        self.read_drive_error()
        self.set_stepper_mode()
        self.get_filament_sensor_data()
        finda = self.get_finda_state()
        self.set_stepper_mode(2)
        if finda:
            msg = ('\tFilament detected! Check unit%i before start printing!'%(self.unit_index))
            logging.info(msg)
            self.gcode.respond(msg)
        self.reset_filament_sensor()

#connect, disconnect
    def connect(self):
        #ch_port = self.mmu2_check_port()
        #if not ch_port:
        #   return False
        try:
            self.mmu2_serial = serial.Serial(self.serial_port,
                                             self.serial_speed,
                                             timeout = self.serial_timeout)
            self.reactor_pause(0.1)
            self.initial_read_and_set()
            self.state['connect'] = ('connected')
            #self.gcode.respond_info('connected %s'%self.state['connect'])
            return True
        except serial.SerialException:
            self.state['connect'] = ('fail')
            return False

    def reconnect_after_fail(self):
        if self.state['connectet'] == ('connected'): return
        if self.mmu2_check_port():
            try:
                self.mmu2_serial = serial.Serial(self.serial_port,self.serial_speed,timeout = self.serial_timeout)
                self.state['connectet'] = ('connected')
            except serial.SerialException:
                self.state['connectet'] = ('SerialException')

    def disconnect(self):
        self.state['connect'] = ('disconnect')
        if self.mmu2_serial == None:
            return
        else:
            self.mmu2_serial.close()

    def mmu2_check_port(self,_timeout = 10):
        if self.state['connect'] == ('connected'):
            return True
        try:
            self.mmu2_serial = serial.Serial(self.serial_port,self.serial_speed,timeout = self.serial_timeout)
            if (self.mmu2_serial.readable() and self.mmu2_serial.writable()):
                return True
        except IOError or self.mmu2_serial.SerialException:
            return False

    def serial_flush(self):
        try:
            self.mmu2_serial.timeout = 0
            flush = self.mmu2_serial.readline()
            self.reactor_pause(0.05)
        except serial.SerialException:
            pass
        return

    def get_unit_index(self):
        return self.unit_index

    def get_print_time(self):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        return print_time

    def reactor_pause(self,time):
        _eventtime = self.reactor.monotonic()
        _eventtime = self.reactor.pause(_eventtime + time)

    def callback_finda(self,eventtime):
        self.state['print_time'] = self.printer.lookup_object('toolhead').get_last_move_time()
        if self.state['cmd_state'] == 'ready' and self.mmu2control.state == ('ready'):
            self.get_finda_state()
            if self.use_pat9125:
                data = self.get_filament_sensor_data()
                self.state['pat9125'] = data
        return eventtime + 2

##################
#MMU memory class#
##################

class mmu2_memory:
    '''
    cte a zapisuje data do souboru pameti MMU2
    read() vraci obsah MMU pameti, pokud soubor pameti neexistuje
        je zalozen novy a navratova hodnota je None
    write() zapisuje data MMU pameti

    pickle verze
    '''
    def __init__(self,_file = '/home/pi/mmu2.mem',data_type = {}):
        self.file = _file
        self.empty = data_type

    def read(self):
        try:
            mem_file=file(self.file,'rb')
            ld = pickle.load(file(self.file,'rb'))
            return ld
        except IOError:
            self.write(self.empty)
            return self.empty
        except KeyError:
            self.write(self.empty)
            return self.empty

    def write(self,data):
            mem_file=file(self.file,'wb')
            pickle.dump(data,mem_file)

#################
#      END      #
#################

def load_config(config):
    return MMU2control(config)