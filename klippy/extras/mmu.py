
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

#Klipper
import homing, kinematics.extruder
import toolhead, gcode, heater, logging
import homing
#generic
import os
import pickle

########################################################################################################################
#MMU2 control class                                                                                                    #
########################################################################################################################

class MMU:
    def __init__(self,config):
            #objects
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.dispay = None
        self.heaters = None
        self.gcode_id = None
            #parameters
        self.enable = self.config.getboolean('enable', False) #M6
            #definition
        self.extruder_group = []
        self.extruder_offset = self.config.getint('extruder_offset', minval=0, default=0)
        self.extruder_groups = self.config.getint('extruder_groups', minval=1, default=1)
        extruders_in_group = config.get('extruders_in_group')  
        self.extruders_in_group = extruders_in_group.split(',')

            #to int
        for q in range (len(self.extruders_in_group)): 
            self.extruders_in_group[q] = int(self.extruders_in_group[q])
            #check consit
        if self.extruder_groups != len(self.extruders_in_group):
            raise homing.EndstopError('inconsistent mmu configuration')
            #generate list   
        num = self.extruder_offset
        for q in range(self.extruder_groups):
            self.extruder_group.append([])
            for k in range(self.extruders_in_group[q]):
                self.extruder_group[q].append(num)
                num += 1
           #check temp
        self.min_temp = self.config.getint('min_temp', minval=180, default=190)
           #temp control for inactive extruder
        self.temp_control = self.config.getboolean('temp_control', False)
        self.extruder_printing_temperature = [0 for x in range(self.extruder_groups)]
           #variables#
        self.state = None
        self.maxval_t = 0
        self.m6_first_fun = None
        for q in range(self.extruder_groups):
            self.maxval_t += len(self.extruder_group[q])
        self.extruder = [None for x in range(self.maxval_t)]
            #MEM for remanent data
        self.mmu2mem = mmu2_memory()
        self.active_tool = None
        self.active_tool_in_group = [None for x in range(self.extruder_groups)]
        self.introduced_filament = [None for x in range(self.maxval_t)]
            #init
        self.init_scripts()
        self.init_commands()
            #M702 used during print [0,0,0....] volatile 
        self.used_during_print = [0 for x in range(self.maxval_t)]

    def handle_ready(self):
        self.state = ('not ready')
        self.init_display()
        self.toolhead = self.printer.lookup_object('toolhead')
        self.heaters = self.printer.lookup_object('heater')

        self.extruder = kinematics.extruder.get_printer_extruders(self.printer)
        logging.info('\textruder_group: %s' %(self.extruder_group)) 
        logging.info('\textruder_groups: %s' %(self.extruder_groups)) 
        logging.info('\textruders_in_group: %s' %(self.extruders_in_group))
        logging.info('\tmaxval T: %s'%(self.maxval_t))
        
        mem = self.restore()
        if mem is {}:
            logging.info('\tmemory file is empty, setting default') 
            self.store()
            
        #logging.info('\tmemory: %s'%(mem)) 
        self.gcode.respond_info('MMU: Ready')
        self.m6_first_fun = True
        if self.active_tool is not None:
            self.cmd_Tn(self.active_tool)
        self.state = ('ready')

    def init_scripts(self):
        '''
        cz:
        Slouzi k nacteni skriptu, bud single nebo list[]
        '''
        self.gcode_before_mmu = self.config.get('gcode_before_mmu', '')
        self.gcode_after_mmu  = self.config.get('gcode_after_mmu', '')
        #self.gcode_if_fail_M6 = self.config.get('gcode_if_fail_m6', '') #zatim nepouzito
        #self.gcode_if_runout = self.config.get('gcode_if_runout', '')   #zatim nepouzito
        
        self.gcode_filament_pullout = self.config.get('gcode_filament_pullout', '') #vytahnout

            #list script
        self.load_unload_profiles = self.config.getint('load_unload_profiles', minval = 1, maxval = self.maxval_t, default=1)
        self.lu_mem = []
        self.gcode_load_profile = []
        self.gcode_unload_profile = []
        for num in range (self.load_unload_profiles):
            self.gcode_load_profile.append(self.config.get(('gcode_load_profile%i' %(num)), ''))
            self.gcode_unload_profile.append(self.config.get(('gcode_unload_profile%i' %(num)), ''))

        self.gcode_old_tool = []
        self.gcode_new_tool = []

        self.gcode_filament_introduce = []
        for tool in range (self.maxval_t):
            self.gcode_old_tool.append(self.config.get(('gcode_new_tool%i' %(tool)), ''))
            self.gcode_new_tool.append(self.config.get(('gcode_old_tool%i' %(tool)), ''))

            self.gcode_filament_introduce = self.config.get(('gcode_filament_introduce%i'%(tool)), '') #zavest  
            self.lu_mem.append(0)

        self.gcode_old_group = []
        self.gcode_new_group = []
        for group in range (self.extruder_groups):
            self.gcode_old_group.append(self.config.get(('gcode_old_group%i_before_m6'%(group)), ''))
            self.gcode_new_group.append(self.config.get(('gcode_new_group%i_after_m6'%(group)), ''))

    def init_commands(self):
            #desc
        desc_M701 = 'M701\n E<num> load filament \n EJECT_EXTRUDER UNIT=unit0 E=0'
        desc_M702 = 'M702\n U<num> unload \nC unload from nozzle \nL<num> load filament to MMU'
        desc_M6 =   'Filament change, like CNC, M6 T unload, M6 T<num> load num filament, M6 T1,2 load multi and set first active'
        desc_QUERY_MMU = ''
        desc_SET_MMU = ''
            #commands prusa MK3s https://github.com/prusa3d/Prusa-Firmware/wiki/Supported-G-codes
        self.gcode.register_command('M701',self.cmd_M701,desc=desc_M701)
        self.gcode.register_command('M702',self.cmd_M702,desc=desc_M702)
           #commands mmu
        self.gcode.register_command('M6',self.cmd_M6,desc=desc_M6)
        self.gcode.register_command('QUERY_MMU',self.cmd_QUERY_MMU,desc=desc_QUERY_MMU)
        self.gcode.register_command('SET_MMU',self.cmd_SET_MMU,desc=desc_SET_MMU)
	   #testing
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
        Provadi vymenu kontrolu parametru pro vymenu filamentu, podle delky listu
        provede bud singleload nebo multiload. Pokud je M6 T 
        '''

        self.state = ('busy')
        
        if ('A') in params:
           self.enable = True
           self.gcode.respond_info('M6 enabled')
           self.set_message('M6 enabled')
           self.state = ('ready')
           return

        if ('D') in params:
            self.enable = False
            self.gcode.respond_info('M6 disabled')
            self.set_message('M6 disabled')
            self.state = ('ready')
            return
            
        if not (self.enable):
            self.gcode.respond_error('MMU M6: not enabled, use <SET_MMU A> or <M6 A> to enable')
            self.set_message('M6 not enabled')
            self.state = ('ready')
            return
                       
        if not ('T') in params:
            self.gcode.respond_error('MMU M6: T missing')
            self.state = ('ready')
            return
            
        self.gcode.respond_info('MMU: M6: prepare')
        self.set_message('Prepare change')       

        if ('T') in params:
            tools = self.get_tool_list_from_t_param_M6(params)
            inactive_temp = (self.min_temp + 10)

            if tools is None:
                self.gcode.respond_info('MMU M6: T<None> unload')
                self.set_message('Unloading filament')
                for tool in self.active_tool_in_group:
                    if tool is not None:
                        tool_group = self.find_tool_group(tool)
                        self.active_tool_in_group[tool_group] = None
                        self.unload_filament_from_extruder(tool)
                        if self.temp_control:
                            self.set_extruder_inactive_temperature(inactive_temp,tool)
                           
                self.active_tool = None
                self.store()
                self.state = ('ready')
                return
                
            if len(tools) == 1:
                self.singleload(tools[0])
                return
            else:
                self.multiload(tools)
                return
           
    def singleload(self,tool):
        '''
        
        cz: zavede jeden filament do hotendu
        '''
        tool_group = self.find_tool_group(tool)
        old_tool_in_group = None
        old_group = None
        
        if self.active_tool is not None:
            old_group = self.find_tool_group(self.active_tool) 
            old_tool_in_group = self.active_tool_in_group[old_group]

        if self.temp_control:
            inactive_temp = (self.min_temp + 10)
            temp_dict = self.get_temp() 
              
        if not self.introduced_filament[tool]: 
            self.gcode.respond_error('MMU M6: error, filament is not introduced') #zmenit na raise exception.......
            self.state = ('ready')
            return         
        
        if tool == self.active_tool:
            self.gcode.respond('MMU M6: T<new> == T<old>')
            self.state = ('ready')
            return
                 
        self.run_script_from_command(self.gcode_before_mmu)
        # pokud je zadany nastroj aktivni v dalsi skupine,snizit teplotu stavajiciho, aktivovat a return
        for ld_tool in self.active_tool_in_group:
            if ld_tool == tool: 
                self.gcode.respond('MMU M6: change group')
                self.run_script_from_command(self.gcode_old_group[old_group])
                self.run_script_from_command(self.gcode_old_tool[old_tool_in_group])                  
                self.run_script_from_command(self.gcode_new_tool[tool])
                self.run_script_from_command(self.gcode_new_group[tool_group])
                self.run_script_from_command(self.gcode_after_mmu)
                self.active_tool = tool
                self.store()
                if self.temp_control and not self.m6_first_fun:# snizit teplotu inaktivni skupiny/skupin
                    for tool in self.active_tool_in_group:
                        if (tool != self.active_tool) and (tool is not None):
                            group = self.find_tool_group(tool)
                            t = ('T%s'%self.extruder_group[group][0])
                            self.extruder_printing_temperature[group] = temp_dict[t]['set'] 
                            self.set_extruder_inactive_temperature(inactive_temp,tool)
                self.m6_first_fun = False            
                return
                
        #pokud je zadany nastroj ve stejne skupine,provest unload                       
        if (tool_group == old_group) and old_group is not None: 
            self.run_script_from_command(self.gcode_old_tool[old_tool_in_group])
            self.run_script_from_command(self.gcode_old_group[old_group])
            self.unload_filament_from_extruder(old_tool_in_group)
            self.active_tool_in_group[old_group] = None
            
        #pokud je zadany nastroj v jine skupine a neni aktivni: provadet unload pokud je obsazeno
        if (tool_group != old_group) and old_group is not None:
            if self.active_tool_in_group[tool_group] is not None:
                self.unload_filament_from_extruder(self.active_tool_in_group[tool_group])
                self.active_tool_in_group[tool_group] = None
            self.run_script_from_command(self.gcode_old_tool[self.active_tool])
            self.run_script_from_command(self.gcode_old_group[old_group]) #v tomto skriptu ~E-10 a zajet do trash/park pozice
            self.active_tool_in_group[old_group] = self.active_tool
            
        #provest load
        self.load_filament_to_extruder(tool)
        #Set variables
        self.active_tool = tool
        self.active_tool_in_group[tool_group] = tool
        #M702 compat
        self.used_during_print[tool] += 1
        #store
        self.store()
        self.run_script_from_command(self.gcode_new_group[tool_group]) #E10 F200...
        self.run_script_from_command(self.gcode_after_mmu)
        #snizit teplotu inactivni skupiny/skupin
        if self.temp_control and not self.m6_first_fun:
            for tool in self.active_tool_in_group:
                if (tool != self.active_tool) and (tool is not None): # and (tool_group != old_group):
                    group = self.find_tool_group(tool)
                    t = ('T%s'%self.extruder_group[group][0])
                    self.extruder_printing_temperature[old_group] = temp_dict[t]['set'] 
                    self.set_extruder_inactive_temperature(inactive_temp,tool)
                    
        #info
        self.gcode.respond_info('SelectExtruder:%s'%(self.active_tool)) #response for repetierServer, rict rep.serveru ze je aktivni jiny nastroj
        self.gcode.respond_info('MMU: Tool change finished, active T:%s'%(self.active_tool))
        self.set_message('Active T:%s'%(self.active_tool))
        self.m6_first_fun = False
        self.state = ('ready')
        return
 
    def multiload(self, tools): 
        '''

        cz:
        Provadi vymenu filamentu:
        M6 T<num>,<num> vymeni/zavede filament <num>,<num> aktivni filament bude 0 v rade
        tato moznost je pouze pro tiskarny ktere maji vice hotendu, tj grup
        
        '''
        #self.gcode.respond_error('Not implemented yet T:%s'%(tools))
        
        if self.temp_control:
            inactive_temp = (self.min_temp + 10)
            temp_dict = self.get_temp()
        
        for tool in tools:    
            if not self.introduced_filament[tool]: 
                self.gcode.respond_error('MMU M6: error, filament is not introduced')
                self.state = ('ready')
                return
        self.gcode.respond_info('>>> multiload 1  tools:%s ' %(tools))
                    
        self.run_script_from_command(self.gcode_before_mmu)
        # provest unload stavajicich T, vyprazdneni jako pri M6 T, 
        # unload se neprovede pokud je zadany nastroj ve skupine aktivnich  
        for tool in self.active_tool_in_group:
           if tool is not None:
               group = self.find_tool_group(tool)
               if tool != self.active_tool_in_group[group]:
                   self.run_script_from_command(self.gcode_old_tool[tool])
                   self.unload_filament_from_extruder(tool)
                   self.active_tool_in_group[group] = None
                
        #jsou prazdne hotendy, load tools, ale v poradi -1,0...
        #pro zjednoduseni zamenit poradi prvku [], pak bude aktivni nastroj posledni v listu. 
        #[1,3] -> [3,1], zobecnit na libovolny pocet
        #aktivovany nastroj bude mit index -1 
        tools.reverse()

        #provest multiload
        for tool in tools:
            group = self.find_tool_group(tool)
            if tool != self.active_tool_in_group[group]:
                self.run_script_from_command(self.gcode_new_tool[tool])
                self.load_filament_to_extruder(tool)
                self.active_tool_in_group[group] = tool
        
        #self.gcode.respond_info('>>>multiload 2  tools:%s ' %(tools))
        #M702 compat
        for tool in tools:
            self.used_during_print[tool] += 1
        #Set variables
        self.active_tool = tools[-1]
        #store
        self.store()
        #
        #self.run_script_from_command(self.gcode_old_group[(self.find_tool_group(tools[0]))] #E10 F200...
        #self.run_script_from_command(self.gcode_new_group[(self.find_tool_group(self.active_tool))]) #E10 F200...
        self.run_script_from_command(self.gcode_after_mmu)
        
        #snizit teplotu inactivnich skupin
        if self.temp_control:
            for tool in tools:
                if tool != self.active_tool:
                    group = self.find_tool_group(tool)
                    t = ('T%s'%(self.extruder_group[group][0]))
                    self.extruder_printing_temperature[group] = temp_dict[t]['set'] 
                    self.set_extruder_inactive_temperature(inactive_temp,tool)
        
        self.gcode.respond_info('SelectExtruder:%s'%(self.active_tool)) #response for repetierServer, rict rep.serveru ze je aktivni jiny nastroj
        self.gcode.respond_info('MMU: Tool change finished, active T:%s'%(self.active_tool))
        self.set_message('Active T:%s'%(self.active_tool))
        self.state = ('ready')
        return

        #self.gcode_filament_pullout = []
        #self.gcode_filament_introduce = []

    def cmd_M701(self, params):
        self.state = ('busy')
        '''
        load filament to MMU
        M701 E<num> or M701 E<num>,<num>
        '''
        if ('E') in params:
            self.run_script_from_command(self.gcode_before_mmu)    
            tools = self.get_tool_list_from_t_param(params, ('E'))
            for tool in tools:
                self.gcode.respond_info('M701: E%s'%(tool))
                self.pulldown_filament(tool)
                self.reactor_pause(2)
            self.run_script_from_command(self.gcode_after_mmu)    
            self.state = ('ready')
            return
        else:
            self.gcode.respond_error('M703: parameter error')
            self.state = ('ready')

    def cmd_M702(self, params):
        self.state = ('busy')
        '''
        ("M702 C")); //unload from nozzle
        ("M702 U")); //used

        M702 Unloads all filaments
        M702 U Unloads all filaments used during print
        M702 C Unloads filament in currently active extruder
        M702 E<num> or E<num>,<num> 

        '''
        self.run_script_from_command(self.gcode_before_mmu)    
        if not (('U') in params) and not (('C') in params) and not(('E') in params):
            for tool in range(self.maxval_t):
                self.pullup_filament(tool)
                self.reactor_pause(2)
            self.run_script_from_command(self.gcode_after_mmu)        
            self.state = ('ready')
            return

        if ('U') in params:
            for tool in range(self.maxval_t):
                if self.used_during_print[tool]:
                    self.pullup_filament(tool)
                    self.used_during_print[tool] = 0
            self.run_script_from_command(self.gcode_after_mmu)            
            self.state = ('ready')
            return

        if ('C') in params:
            tools = self.get_tool_list_from_t_param(_params, ('C'))
            if tool == None:
               self.pullup_filament(self.active_tool)
               self.reactor_pause(2)
            for tool in tools:
               self.pullup_filament(tool)
               self.reactor_pause(2)
            self.run_script_from_command(self.gcode_after_mmu)       
            self.state = ('ready')
            return
            
        if ('E' in params):
            tools = self.get_tool_list_from_t_param(params, ('E'))
            for tool in tools:
                self.gcode.respond_info('M702: E%s'%(tool))
                self.pullup_filament(tool)
                self.reactor_pause(2)
            self.run_script_from_command(self.gcode_after_mmu)       
            self.state = ('ready')
            return    
                

    def cmd_SET_MMU(self,params):
        self.gcode.respond_info(str(params))
        if ('T' in params): #set active tool
            if params['T'] == ('') or None:
                #kdyz je T<None> nastavit None
                for q in range(self.extruder_groups):
                    self.active_tool_in_group[q] = None
                self.active_tool = None
            else:
                T_param = self.gcode.get_int('T', params, minval = 0, maxval = self.maxval_t)
                self.active_tool = T_param
                group = self.find_tool_group(T_param)
                self.active_tool_in_group[group] = T_param
                self.introduced_filament[T_param] = True
            self.gcode.respond_info('SET_MMU: Active T: %s '%(self.active_tool))

        elif ('P') in params and ('T') in params:
            '''
            #MMU_SET P1 T0 > profile 1 to T0
            '''
            T_param = self.gcode.get_int('T', params, minval=0, maxval=self.maxval_t)
            P_param = self.gcode.get_int('P', params, minval=0, maxval=self.maxval_t)
            self.lu_mem[T_param] = P_param
            self.gcode.respond_info('SET_MMU: Set LU profile: % tool: %'%(P_param,T_param))
            
        elif ('I') in params: 
            _params = params           
            tools = self.get_tool_list_from_t_param(_params, ('I'))
            
            for tool in tools:
                if ('R') in params:
                    self.introduced_filament[tool] = False
                else:    
                    self.introduced_filament[tool] = True
                    
            self.gcode.respond_info('SET_MMU: Introduced_filament:%s' %(self.introduced_filament))    
            
        if ('S') in params:
            self.gcode.respond_info('SET_MMU: Store')
            self.store()

    def cmd_QUERY_MMU(self,params):
        if self.enable: m6_stat = ('enabled')
        else: m6_stat = ('disabled')

        self.gcode.respond('QUERY_MMU: \n')
        self.gcode.respond('\tM6: %s'%(m6_stat))
        tool = ('None')
        if self.active_tool != None: tool = str(self.active_tool)
        self.gcode.respond(('\tActive tool: %s')%(tool))
        self.gcode.respond(('\tActive tool in group: %s')%(self.active_tool_in_group))
        self.gcode.respond(('\tL/U memory: %s')%(self.lu_mem))
        self.gcode.respond(('\tUsed during print: %s')%(self.used_during_print))
        self.gcode.respond(('\tIntroduced filament: %s')%(self.introduced_filament))

    def cmd_TEST0(self,params):
        eventtime = self.reactor.monotonic()
        '''
        na volne testovani0
        '''
        self.gcode.respond_info('<<<TEST0 START>>>')
        #
        
        #
        self.gcode.respond_info('<<<TEST0 STOP>>>')
        return

    def cmd_TEST1(self,params):
        '''
        na volne testovani1
        '''
        self.gcode.respond_info('<<<TEST1 START>>>')
        #

        #
        self.gcode.respond_info('<<<TEST1 STOP>>>')
        return

####
    def find_tool_group(self,tool):
        if tool == None: return None

        if self.extruder_groups == 1:
            group = 0
        else:
            try:
               for group in range(self.extruder_groups):
                   if tool in (self.extruder_group[group]):
                       break
               else:
                   group = None
            except IndexError:
                group = None
            
        return group
####
    def get_tool_list_from_t_param_M6(self,params):
        '''
        cz:
        pro budouci implementaci multiloadu
        z params =  {'M6': '0', '#command': 'M6', '#original': 'M6 T1,6', 'T': '1,6'}
        extrahuje tool = [1,6]
        tool[0] nastroj(1)  bude aktivni po M6
        load se provadi v od konce seznamu, tj t[-1] t[0]
        
        '''
        tools = self.get_tool_list_from_t_param(params,('T'))
        if tools is None:
            return None
        
        if len(tools) > self.extruder_groups:
           self.gcode.respond_error('MMU: Multitool tools > groups')
           return None
         
        diff = 0
        for tool in (tools):
            diff = self.find_tool_group(tool) - diff
        if diff == 0 and len(tools) > 1:
            self.gcode.respond_error('MMU: Multitool parameter T[0] T[1] = same group')
            return None
        
        #doplnit 
        #if len(tools) == 1:
        #   for q in range(self.extruder_groups - 1):
        #       tools.append(None)
                  
        #self.find_tool_group
        return tools
        
    def get_tool_list_from_t_param(self, _params, char = ('T')):
        '''
        
        cz: vraci seznam nastroju, pro multiload
        '''
        tools = _params[char]
        if tools == (''):
            return None
        toolstring = tools.split(',')
        tool_list = []
        for q in (toolstring):
            try:
                if (int(q)) > self.maxval_t:
                    self.gcode.respond_error('MMU: Multitool parameter %i error, too high'%q)
                    return None
                if (int(q)) < self.extruder_offset:
                    self.gcode.respond_error('MMU: Multitool parameter %i error, too low'%q)
                    return None 
                
                tool_list.append(int(q))
            except ValueError:
                self.gcode.respond_error('MMU: Multitool parameter error')
                
        #tool_list = (list(dict.fromkeys(tool_list))) #remove duplicate 
                    
        if len(tool_list) > self.maxval_t:
            self.gcode.respond_error('MMU: Multitool parameter is grater than maxval_t')
            return None
        else:
            return tool_list    
        
    def get_int_value_from_param(self,_params,key):
        is_key = False
        num = None
        if (key) in (_params):
            is_key = True
            try:
                num = _params[key]
                if len(num) == 0: num = None
                else: num = int(num)
            except ValueError:
                self.gcode.respond_error('MMU2: Error in parameter: %s key: %s' %(_params,key))
                return None, None
        return is_key,num

#extruder temperature
    def check_extruder_temperature(self,group_num):
       '''
       
       cz:
       testuje teplotu extruderu pro vymenu
       '''
       tool = ('T%s' %self.extruder_group[group_num][0])
       temp_dict = self.get_temp()   
       temp = temp_dict[tool]['read']
       
       if temp < self.min_temp:
           self.gcode.respond_error('MMU: Extruder mintemp mintemp:%s measured:%s' %(self.min_temp,temp))
           self.set_message('Extr. mintemp')
           raise homing.EndstopError('MMU: Extruder mintemp:%s measured:%s'%(self.min_temp,temp))
           
    def get_temp(self, _rnd = 0):
        temp_dict = {}
	'''
	
	cz:
	Vraci teploty
	'''
        for gcode_id, sensor in sorted(self.heaters.get_gcode_sensors()):
            eventtime = self.reactor.monotonic()
            cur, target = sensor.get_temp(eventtime)
            temp_dict[gcode_id] = {'read':round(cur,_rnd),'set':target}
        return temp_dict
        
    def set_extruder_inactive_temperature(self, temp,tool):
        temp_dict = self.get_temp()
        group = self.find_tool_group(tool)
        t = ('T%s'%(self.extruder_group[group][0]))
        if temp_dict[t]['set'] == 0:
            return
        else:    
            params = {'S':temp,'T':tool}
            self.gcode._set_temp(params)
        
    def set_extruder_printing_temperature(self, temp,tool):
        params = {'S':temp,'T':tool}
        self._set_temp(params, wait=True)
        
    def reduce_temperature(self):
        return
        
#load/unload to extruder

    def load_filament_to_extruder(self,tool):
        self.wait_for_finish_scripts()
        self.cmd_Tn(tool)
        tool_group = self.find_tool_group(tool)
        self.check_extruder_temperature(tool_group)
        
        try: profile_num = self.lu_mem[tool]
        except IndexError: profile_num = 0

        self.gcode.respond_info("MMU: Load filament to extruder")
        self.run_script_from_command(self.gcode_load_profile[profile_num])


    def unload_filament_from_extruder(self,tool):
        self.wait_for_finish_scripts()
        self.cmd_Tn(tool)
        tool_group = self.find_tool_group(tool)
        self.check_extruder_temperature(tool_group)
        
        try: profile_num = self.lu_mem[tool]
        except IndexError: profile_num = 0

        self.gcode.respond_info("MMU: Unloading filament")
        self.run_script_from_command(self.gcode_unload_profile[profile_num])
        self.wait_for_finish_scripts()
        
    def wait_for_finish_scripts(self):
        self.toolhead.wait_moves()

    def pulldown_filament(self, tool):
        '''
        
        cz: zavadi filament
        pokud je jiz dle pameti pritomen, nic se neprovede
        '''
        if self.introduced_filament[tool] is True:
            return
            
        self.wait_for_finish_scripts()    

        group = self.find_tool_group(tool)
        self.check_extruder_temperature(group)
        self.cmd_Tn(tool)
        self.reactor_pause(2)
        self.run_script_from_command(self.gcode_filament_introduce[tool])

        self.introduced_filament[tool] = True
        self.store()
        self.extruder_motor_off(tool) 
        return

    def pullup_filament(self, tool):
        '''
        
        cz: vytahuje filament z extruderu, 
        pokud je zaroven aktivni jako nastroj, 
        provede se jeho vytazeni z hotendu podle L/U profilu
        nakonec vypne motor extruderu
        '''
        if self.introduced_filament[tool] is False:
            return
        if self.active_tool == tool:
           self.gcode.respond_error("MMU: Unload filament from hotend first")
           return    
        self.wait_for_finish_scripts()
        group = self.find_tool_group(tool)
        self.check_extruder_temperature(group)
        self.cmd_Tn(tool)
        self.reactor_pause(1)
            
        self.run_script_from_command(self.gcode_filament_pullout)
        self.introduced_filament[tool] = False
        self.store()
        self.extruder_motor_off(tool) 
        return
        
    def run_script_from_command(self,script):
        self.gcode.run_script_from_command(script)
        self.wait_for_finish_scripts()
#Memory    
    def restore(self):
        mem = self.mmu2mem.read()
        if ('active_tool') in mem: self.active_tool = mem['active_tool']
        if ('active_tool_in_group') in mem: self.active_tool_in_group = mem['active_tool_in_group']   
        if ('lu_mem') in mem: self.lu_mem = mem['lu_mem'] 
        if ('introduced_filament') in mem: self.introduced_filament = mem['introduced_filament'] 
    
    def store(self):
        mem = {'active_tool': self.active_tool,'active_tool_in_group' : self.active_tool_in_group,'lu_mem': self.lu_mem,'introduced_filament':self.introduced_filament}
        logging.info('MMU: store: %s'%(mem))
        self.mmu2mem.write(mem)

    def reset_store(self):
        mem = {}
        logging.info('MMU store: RESET')
        self.mmu2mem.write(mem)
#gcode Tn
    def cmd_Tn(self, tool):
        #self.gcode.respond_info('>>>cmd_Tn:%i'%tool)
        #without call script
        #extruders = kinematics.extruder.get_printer_extruders(self.printer)

        self.toolhead.set_extruder(self.extruder[tool])
        self.gcode.extruder = self.extruder[tool]
        self.gcode.reset_last_position()
        self.gcode.extrude_factor = 1.
        self.gcode.base_position[3] = self.gcode.last_position[3]
        self.gcode.actual_tool = tool
        self.gcode.active_extruder = tool
    def get_print_time(self):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        return print_time

    def reactor_pause(self,time):
        self.reactor.pause((self.reactor.monotonic()) + time)
        
    #LCD
    def set_message(self,msg,msg_time = None):
        if self.display != None:
            self.display.set_message(msg,msg_time)
    #stepper
    def extruder_motor_off(self,motor_num):
        time = self.get_print_time()
        self.extruder[motor_num].stepper.motor_enable(time, 0)
        self.extruder[motor_num].need_motor_enable = True
        self.reactor_pause(0.1)

    def stats(self, eventtime):
        if self.state:
           return False, '%s: tool:%s active tool in group:%s lu_mem:%s introduced filament:%s' % ('MMU',
           self.active_tool,self.active_tool_in_group,self.lu_mem,self.introduced_filament)
           
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
    return MMU(config)