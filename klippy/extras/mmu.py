
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

#Klipper
import homing, kinematics.extruder
import toolhead, gcode, heater, logging
import homing, stepper
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
        self.printer.register_event_handler("extruder:activate_extruder", self.handle_activate_extruder)
        self.heaters = None
        self.gcode_id = None
            #parameters           
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
        self.min_temp = self.config.getint('min_temp', minval=190, default=190)
           #temp control for inactive extruder
        self.temp_control = self.config.getboolean('temp_control', False)
        self.extruder_printing_temperature = [0 for x in range(self.extruder_groups)]
           #variables#
        self.state = None
        self.maxval_t = 0
        self.m6_first_run = None
        for q in range(self.extruder_groups):
            self.maxval_t += len(self.extruder_group[q])
        self.extruder = [None for x in range(self.maxval_t)]
            #MEM for remanent data
        self.mmu2mem = mmu2_memory()
        self.active_tool = None
        self.active_tool_in_group = [None for x in range(self.extruder_groups)]
        self.introduced_filament = [None for x in range(self.maxval_t)]
            #init
        self.init_single_script()
        self.init_list_scripts()
        self.init_commands()

    def handle_ready(self):
        self.state = ('not ready')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.heaters = self.printer.lookup_object('heater')

        #self.extruder = kinematics.extruder.get_printer_extruders(self.printer)
        logging.info('\textruder_group: %s' %(self.extruder_group)) 
        logging.info('\textruder_groups: %s' %(self.extruder_groups)) 
        logging.info('\textruders_in_group: %s' %(self.extruders_in_group))
        logging.info('\tmaxval T: %s'%(self.maxval_t))
        
        mem = self.restore()
        if mem is {}:
            logging.info('\tmemory file is empty, setting default') 
            self.store()
            
        logging.info('\tmemory: %s'%(mem)) 
        
        self.m6_first_run = True
        if self.active_tool is not None:
            self.activate_extruder(self.active_tool)
        self.state = ('ready')

        self.gcode.respond_info('MMU: Ready')

    def handle_activate_extruder(self):
        return None    

    def init_list_scripts(self):
        #generovani scriptu, pull intr
        self.intr_pull_length = self.get_int_list_from_string(self.config.get('intr_pull_length', ''))
        self.intr_pull_feed = self.config.getint('intr_pull_feed',  minval=10, default=1000)

        bf_s = 'M83\nG92 E0\n'
        af_s = 'G92 E0\n'

        self.gcode_filament_introduce = []
        self.gcode_filament_pullout = []
        #
        for q in range(self.maxval_t):
            self.gcode_filament_introduce.append(self.gen_script(self.intr_pull_length[q],self.intr_pull_feed,af_scr = af_s, bf_scr = bf_s))  
            self.gcode_filament_pullout.append(self.gen_script(-self.intr_pull_length[q],self.intr_pull_feed,af_scr = af_s, bf_scr = bf_s)) 
        #generovani scriptu, M6 load/unload
        self.gcode_load = None
        self.gcode_unload = None
        # 
        load_length = self.get_int_list_from_string(self.config.get('load_length',''))
        unload_length = self.get_int_list_from_string(self.config.get('unload_length',''))
        unload_feed = self.get_int_list_from_string(self.config.get('unload_feed',''))
        load_feed = self.get_int_list_from_string(self.config.get('load_feed',''))
        #
        self.gcode_load = self.gen_script_m6(load_length,load_feed,af_scr = af_s, bf_scr = bf_s)
        self.gcode_unload = self.gen_script_m6(unload_length,unload_feed,af_scr = af_s, bf_scr = bf_s)
        #
        self.gcode_old_tool = []
        self.gcode_new_tool = []
        for tool in range (self.maxval_t):
            self.gcode_old_tool.append(self.config.get(('gcode_new_tool%i' %(tool)), ''))
            self.gcode_new_tool.append(self.config.get(('gcode_old_tool%i' %(tool)), ''))
        self.gcode_old_group = []
        self.gcode_new_group = []
        for group in range (self.extruder_groups):
            self.gcode_old_group.append(self.config.get(('gcode_old_group%i_before_m6'%(group)), ''))
            self.gcode_new_group.append(self.config.get(('gcode_new_group%i_after_m6'%(group)), ''))

        logging.info(self.gcode_load,self.gcode_filament_pullout)   

    def init_single_script(self):
        self.gcode_before_m6 = self.config.get('gcode_before_mmu', '')
        self.gcode_after_m6  = self.config.get('gcode_after_mmu', '')
        #
        self.gcode_before_intr_pull = self.config.get('gcode_before_intr_pull', '')
        self.gcode_after_intr_pull  = self.config.get('gcode_after_intr_pull', '')

    def get_int_list_from_string(self, istring):
        istring_list = istring.split(",")
        for q in range(len(istring_list)):
            try:
                istring_list[q] = int(istring_list[q])
            except ValueError:
                logging.error("MMU: value %s index:%i not int" %(c[q],q))
        return istring_list

    def gen_script(self,length,feed,bf_scr = None,af_scr = None,
        axis = 'E',
        ramp_data_feed = [0.2,0.5,1,0.5],
        ramp_data_length = [0.1,0.1,0.6,0.2]):

        line_end = '\n'
        cmd_linear_move = 'G1'
        lng_ramp_data = len(ramp_data_feed)  
        line_script = []
        script = None

        if len(ramp_data_feed) != len(ramp_data_length):
            logging.error("MMU: wrong ramp data length")
            return None 
        elif sum(ramp_data_length) != 1.0:
            logging.error("MMU: wrong sum")
            return None                        
        if bf_scr: line_script.append(bf_scr)
        for q in range(lng_ramp_data):
            line_script.append('%s %s%0.1f F%i %s'%(cmd_linear_move,axis,(ramp_data_length[q] * length),(int(ramp_data_feed[q] * feed)),line_end))
        if af_scr: line_script.append(af_scr)

        script = ''.join(line_script)
        return script

    def gen_script_m6(self,length_list,feed_list,bf_scr = None,af_scr = None,axis = 'E'):
        
        line_end = '\n'
        cmd_linear_move = 'G1'
        lng_length_list = len(length_list)  
        line_script = []
        script = None

        if len(length_list) != len(feed_list):
            print("wrong data length")
            return None             
        if bf_scr: line_script.append(bf_scr)
        for q in range(lng_length_list):
            #G1 E<length> F
            line_script.append('%s %s%0.1f F%i %s'%(cmd_linear_move,axis,length_list[q],feed_list[q],line_end))
        if af_scr: line_script.append(af_scr)

        script = ''.join(line_script)
        return script

    def init_commands(self):
            #desc
        desc_M701 = 'M701\n T<num>'
        desc_M702 = 'M702\n T<num>'
        desc_M6 =   'Filament change, M6 T-1 unload, M6 T<num>'
        desc_QUERY_MMU = ''
        desc_SET_MMU = ''

        self.gcode.register_command('M701',self.cmd_M701,desc=desc_M701)
        self.gcode.register_command('M702',self.cmd_M702,desc=desc_M702)
       #commands mmu
        self.gcode.register_command('M6',self.cmd_M6,desc=desc_M6)
        self.gcode.register_command('QUERY_MMU',self.cmd_QUERY_MMU,desc=desc_QUERY_MMU)
        self.gcode.register_command('SET_MMU',self.cmd_SET_MMU,desc=desc_SET_MMU)
	   #testing
        self.gcode.register_command('TEST0',func = self.cmd_TEST0,desc=None)
        self.gcode.register_command('TEST1',self.cmd_TEST0,desc=None)

#commands
    def cmd_M6(self,params):

        self.state = ('busy')
        tool = None 
                           
        if not ('T') in params:
            self.gcode.respond_error('MMU M6: <Tnum> missing')
            self.state = ('ready')
            return

        self.gcode.respond_info('MMU: M6: prepare')

        if ('T') in params:
            tool = self.gcode.get_int('T', params, minval = -1, maxval = self.maxval_t)

            inactive_temp = (self.min_temp + 10)
            if tool == -1:
                self.gcode.respond_info('MMU M6: T<None> unload')
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
            self.gcode.respond_info('MMU: M6 begin')
            self.singleload(tool)
            return
           
    def singleload(self,tool):

        if tool == self.active_tool:
            self.gcode.respond('MMU: M6: T<new> == T<old>')
            self.state = ('ready')
            return

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
            self.gcode.respond_error('MMU: M6: error, filament is not introduced') #zmenit na raise exception.......
            self.state = ('ready')
            return         
                       
        self.run_script_from_command(self.gcode_before_m6)
        # pokud je zadany nastroj aktivni v dalsi skupine,snizit teplotu stavajiciho, aktivovat a return

        for ld_tool in self.active_tool_in_group:
            if ld_tool == tool: 
                self.gcode.respond('MMU M6: change group')
                self.run_script_from_command(self.gcode_old_group[old_group])
                self.run_script_from_command(self.gcode_old_tool[old_tool_in_group])                  
                self.run_script_from_command(self.gcode_new_tool[tool])
                self.run_script_from_command(self.gcode_new_group[tool_group])
                self.run_script_from_command(self.gcode_after_m6)
                self.active_tool = tool
                self.store()
                if self.temp_control and not self.m6_first_run:# snizit teplotu inaktivni skupiny/skupin
                    for tool in self.active_tool_in_group:
                        if (tool != self.active_tool) and (tool is not None):
                            group = self.find_tool_group(tool)
                            t = ('T%s'%self.extruder_group[group][0])
                            self.extruder_printing_temperature[group] = temp_dict[t]['set'] 
                            self.set_extruder_inactive_temperature(inactive_temp,tool)
                self.m6_first_run = False            
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
        #store
        self.store()
        self.run_script_from_command(self.gcode_new_group[tool_group]) #E10 F200...
        self.run_script_from_command(self.gcode_after_m6)
        #snizit teplotu inactivni skupiny/skupin
        if self.temp_control and not self.m6_first_run:
            for tool in self.active_tool_in_group:
                if (tool != self.active_tool) and (tool is not None): # and (tool_group != old_group):
                    group = self.find_tool_group(tool)
                    t = ('T%s'%self.extruder_group[group][0])
                    self.extruder_printing_temperature[old_group] = temp_dict[t]['set'] 
                    self.set_extruder_inactive_temperature(inactive_temp,tool)
                    
        #info
        self.gcode.respond_info('SelectExtruder:%s'%(self.active_tool)) #response for repetierServer, rict rep.serveru ze je aktivni jiny nastroj
        self.gcode.respond_info('MMU: M6: Tool change finished, active T:%s'%(self.active_tool))
        self.m6_first_run = False
        self.state = ('ready')
        return
 
    def cmd_M701(self, params):
        self.state = ('busy')
        '''
        M701 T<num>
        '''
        if (('T')) in params:
            tool = self.gcode.get_int('T', params, minval = 0, maxval = self.maxval_t)
            self.run_script_from_command(self.gcode_before_intr_pull)
            if self.introduced_filament[tool] is True:
                self.gcode.respond_info('MMU: M701: Filament E:%s is indroduced' %(tool))
            else:
                self.gcode.respond_info('MMU: M701: Introducing E%s'%(tool))    
                self.pulldown_filament(tool)
                self.reactor_pause(2)
            self.run_script_from_command(self.gcode_after_intr_pull)
            #restore active
            if self.active_tool is not None:
                self.activate_extruder(self.active_tool)
            self.state = ('ready')
            return
        else:
            self.gcode.respond_error('MMU: M701: parameter error')
            self.state = ('ready')

    def cmd_M702(self, params):
        self.state = ('busy')
        '''
        M702 T<num>  
        '''
        self.run_script_from_command(self.gcode_before_intr_pull)
                    
        if ('T' in params):
            tool = self.gcode.get_int('T', params, minval = 0, maxval = self.maxval_t)
            if self.introduced_filament[tool] is False:
                self.gcode.respond_info('MMU: M702: Filament T:%i is not introduced' %(tool))             
            if self.active_tool == tool:
                self.unload_filament_from_extruder(tool)
                self.active_tool_in_group[self.find_tool_group(tool)] = None
                self.active_tool = None 
            else:    
                self.gcode.respond_info('MMU: M702: T%s'%(tool))
                self.pullup_filament(tool)
                self.reactor_pause(1)
            self.run_script_from_command(self.gcode_after_intr_pull)
            #restore active
            if self.active_tool is not None:
                self.activate_extruder(self.active_tool)       
            self.state = ('ready')
            return
        else:
            self.gcode.respond_error('MMU: M702: parameter error')
            self.state = ('ready')    
                
    def cmd_SET_MMU(self,params):
        self.gcode.respond_info(str(params))

        if ('T' in params): #set active tool
            tool = self.gcode.get_int('T', params, minval = -1, maxval = self.maxval_t)
            if tool = -1:
                for q in range(self.extruder_groups):
                    self.active_tool_in_group[q] = None
                self.active_tool = None
            else:   
                self.active_tool = tool
                group = self.find_tool_group(tool)
                self.active_tool_in_group[group] = tool
                self.introduced_filament[tool] = True
            self.gcode.respond_info('MMU: SET_MMU: Active T: %s '%(self.active_tool))
            
        elif ('I') in params: 
            tool = params['I']
            if ('R') in params:
                self.introduced_filament[tool] = False
            else:    
                self.introduced_filament[tool] = True         
            self.gcode.respond_info('MMU: SET_MMU: Introduced_filament:%s' %(self.introduced_filament))    
            
        if ('S') in params:
            self.gcode.respond_info('MMU: SET_MMU: Store')
            self.store()

    def cmd_QUERY_MMU(self,params):

        self.gcode.respond('QUERY_MMU: \n')
        tool = ('None')
        if self.active_tool != None: tool = str(self.active_tool)
        self.gcode.respond(('\tActive tool: %s')%(tool))
        self.gcode.respond(('\tActive tool in group: %s')%(self.active_tool_in_group))
        self.gcode.respond(('\tIntroduced filament: %s')%(self.introduced_filament))

    def cmd_TEST0(self,params):
        eventtime = self.reactor.monotonic()
        '''
        na volne testovani0
        '''
        self.gcode.respond_info('<<<TEST0 START>>>')
        self.gcode.respond_info(str(params))
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
        self.activate_extruder(tool)
        self.check_extruder_temperature(self.find_tool_group(tool))
        self.gcode.respond_info("MMU: Load filament to extruder")
        self.run_script_from_command(self.gcode_load)

    def unload_filament_from_extruder(self,tool):
        self.activate_extruder(tool)
        self.check_extruder_temperature(self.find_tool_group(tool))        
        self.gcode.respond_info("MMU: Unloading filament")
        self.run_script_from_command(self.gcode_unload)
        
    def pulldown_filament(self, tool):
        self.check_extruder_temperature(self.find_tool_group(tool))
        self.activate_extruder(tool)
        self.reactor_pause(1)
        self.run_script_from_command(self.gcode_filament_introduce[tool])
        self.introduced_filament[tool] = True
        self.store()
        return

    def pullup_filament(self, tool):
        self.check_extruder_temperature(self.find_tool_group(tool))
        self.activate_extruder(tool)
        self.reactor_pause(1)     
        self.run_script_from_command(self.gcode_filament_pullout[tool])
        self.introduced_filament[tool] = False
        self.store()
        return
        
    def run_script_from_command(self,script):
        logging.info(script)
        self.gcode.run_script_from_command(script)
        self.wait_for_finish_scripts()

    def wait_for_finish_scripts(self):
        self.toolhead.wait_moves()    
#Memory    
    def restore(self):
        mem = self.mmu2mem.read()

        if ('active_tool') in mem: self.active_tool = mem['active_tool']
        if ('active_tool_in_group') in mem: self.active_tool_in_group = mem['active_tool_in_group']
        if ('introduced_filament') in mem: self.introduced_filament = mem['introduced_filament'] 
    
    def store(self):
        mem = {'active_tool': self.active_tool,'active_tool_in_group' : self.active_tool_in_group,'introduced_filament':self.introduced_filament}
        logging.info('MMU: store: %s'%(mem))
        self.mmu2mem.write(mem)

    def reset_store(self):
        mem = {}
        logging.info('MMU store: RESET')
        self.mmu2mem.write(mem)
#
    def activate_extruder(self,tool):
        if tool == 0:
            extruderstring = "extruder"
        else:
            extruderstring = ("extruder%i" %(tool))
        self.run_script_from_command(("ACTIVATE_EXTRUDER EXTRUDER=%s"%(extruderstring)))
        #self.toolhead.flush_step_generation()
        #self.toolhead.set_extruder(tool, 0)
        logging.info("MMU: Activating extruder %i" %(tool))
        #
    def get_print_time(self):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        return print_time

    def reactor_pause(self,time):
        self.reactor.pause((self.reactor.monotonic()) + time)
        
    def stats(self, eventtime):
        if self.state:
           return False, '%s: tool:%s active tool in group:%s introduced:%s' % ('MMU',
           self.active_tool,self.active_tool_in_group,self.introduced_filament)
           
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
            #return None
            return self.empty
        except KeyError:
            self.write(self.empty)
            #return None
            return self.empty

    def write(self,data):
            mem_file=file(self.file,'wb')
            pickle.dump(data,mem_file)

#################
#      END      #
#################

def load_config(config):
    return MMU(config)