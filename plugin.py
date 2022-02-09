# Manage a MCP23017 I2C Listener
# $Id: plugin.py 187 2022-02-09 11:27:04Z eric $
#
# idx_cmd to execute given switch on click and change it's state
# idx_only_cmd to execute given switch on click only
# on_delay to keep icon status On during X cycles (if value return takes ime
# off_delay

"""
<plugin key="MCP23017_listener" name="MCP23017 Listener plugin" author="Morand" version="1.0.0" wikilink="" externallink="">
    <description>
        <h2>MCP23017</h2><br/>
          Manage your MCP23017 for input in read mode.<br />
          Use mask to select the IO you want to use.<br />
           NEED Domoticz 4.9788 or higher
        <h3>Parameters</h3><br />
        <ul>
          <li>Address: I2C address of MCP23017 chip, IP Address of MQTT server if you want to enable MQTT Remote Domoticz</li>
          <li>Port: MQTT server port for MQTT Remote Domoticz</li>
          <li>Mask Port: Set IO uses. Example: 0b0001001,0b00000001 on port A to use only IO A0 and A3 values and A0 values with inverted values</li>
          <li>Interruption: Enable interruption. ONLY FOR Raspberry!</li>
          <li>GPIO Number for IT: Set the RPi GPIO for catching IT. It is a comma separated list: first value for Port A, second (optional) for port B</li>
          <li>Polling intervall: delay between two read command in seconds. Possible values are 1 to 20 or 30,40,50... . If 0 no pooling </li>
        </ul>
   </description>
    <params>
      <param field="Address" label="Address" width="150px" default='0x20'/>
      <param field="Port" label="MQTT Remote Domoticz" width="300px" default=''/>
      <param field="Mode1" label="Mask Port A" width="170px" default='0b00000000,0b00000000'/>
      <param field="Mode2" label="Mask Port B" width="170px" default='0b00000000,0b00000000'/>
      <param field="Mode3" label="Interruption" width="150px">
         <options>
           <option label="Port A" value="4"/>
           <option label="Port B" value="2"/>
           <option label="Port A and B" value="6"/>
           <option label="Mirror Port A and B" value="7"/>
           <option label="None" value="0" default="true"/>
          </options>
      </param>
      <param field="Mode4" label="GPIO Number for IT" width="150px" default='20,21'/>
      <param field="Mode5" label="Polling interval" width="150px" default='3'/>
      <param field="Mode6" label="Debug" width="75px">
            <options>
                <option label="True" value="Debug"/>
                <option label="False" value="Normal"  default="true" />
            </options>
       </param>
    </params>
</plugin>
"""

import Domoticz
import sys
import time
import os
import binascii
import socket
import _thread
sys.path.append('/usr/lib/python3/dist-packages')
import smbus
from mqtt import MqttClientSH2
import json

class MCP23017Plugin:
    IODIRA  = 0x00 # Pin direction register for port A
    IODIRB  = 0x01 # Pin direction register for port B
    IOPOLA  = 0x02 # Invert polarity for port A
    IOPOLB  = 0x03 # Invert polarity for port B
    GPINTENA= 0x04 # Enable interrupt for GPIO Port A
    GPINTENB= 0x05 # Enable interrupt for GPIO Port B
    DEFVALA = 0x06 # Reference values for the interruptions on port A
    DEFVALB = 0x07 # Reference values for the interruptions on port B
    INTCONA = 0x08 # Interruption mode -- 1:compare to DEFVAL, 0:compare to its old value
    INTCONB = 0x09 # Interruption mode -- 1:compare to DEFVAL, 0:compare to its old value
    INTCAPA = 0x10 # value stored after an interruption
    INTCAPB = 0x11 # value stored after an interruption
    GPIOA   = 0x12 # GPA register for input
    GPIOB   = 0x13 # GPA register for input
    IOCONA  = 0x0A # IO connection Port A
    IOCONB  = 0x0B # IO connection Port B
    GPPUA   = 0X0C # Pullup resistor Port A
    GPPUB   = 0X0D # Pullup resistor Port B
    INTFA   = 0x0E #
    INTFB   = 0x0F #
    
    enabled = False
    
    def __init__(self):
        self._mask=None
        self._lastState=[0b00000000,0b00000000]
        self._addr=None
        self._fullIODir=''
        self._poolingTime=0
        self._pooling=True
        self._beat=0
        self._configFilename=None
        #self._revertList=[]
        #PortA, PortB, Mirror
        self._it=0
        self.mqttClient=None
        self._slaveMode=False
        self._inTransition={}
        self._itLogger={}
        self._logLevel=['Debug','Log','Status','Error']
        for level in self._logLevel:
            self._itLogger[level]=[]
        return

    def onStart(self):
        if Parameters["Mode6"] == "Debug":
            Domoticz.Debugging(1)
        Domoticz.Log("onStart called")
        #MQTT if enabled
        if len(Parameters["Address"].split(','))>=2:
            import socket
            hostname=socket.gethostname().split('.')[0]
            self.base_topic = 'domoticz/slaves/'+hostname
            self.mqttserveraddress = Parameters["Address"].strip().split(',')[1]
            self.mqttserverport = Parameters["Port"].strip()
            Domoticz.Status("MQTT Domoticz Remote enabled on:"+self.base_topic+", "+self.mqttserveraddress+":"+self.mqttserverport)
            self.mqttClient = MqttClientSH2(self.mqttserveraddress, self.mqttserverport, "", self.onMQTTConnected, self.onMQTTDisconnected, self.onMQTTPublish, self.onMQTTSubscribed)
            if len(Parameters["Address"].split(','))==3 and Parameters["Address"].split(',')[2]=='no_slave_mode':
                pass
            else:
                self.slaveMode=True

        
        DumpConfigToLog()
        self._bus=smbus.SMBus(1)
        self._mask=[int(Parameters["Mode1"].split(',')[0],2),int(Parameters["Mode2"].split(',')[0],2)]
        self._inversionMask=[int(Parameters["Mode1"].split(',')[1],2),int(Parameters["Mode2"].split(',')[1],2)]
        self._addr=int(Parameters["Address"].split(',')[0],16)
        self._fullIODir=(Parameters["Mode2"][2:10]+Parameters["Mode1"][2:10])[::-1]
        toUpdate=[]
        #Update devices which have changed
        for i in range (1,17):
            if self._fullIODir[i-1]=='0' and i in Devices :
                Devices[i].Delete()
            elif self._fullIODir[i-1]=='1' and not i in Devices:
                    toUpdate.append(i)
        self.__createDevice(toUpdate)
        #Initialise ports
        self._it=int(Parameters["Mode3"])
        it=[False,False]
        if self._it>=4:
            it[0]=True
        if self._it>=2 and self._it!=4:
            it[1]=True
        for i in range(0,2):
            if(self._mask[i] > 0):
                self.__initPort(i,it[i])
                self.updateState(i)
        #It mirroring
        if self._it==7:
            Domoticz.Debug("Activate Interruption mirroring")
            self._bus.write_byte_data(self._addr, MCP23017Plugin.IOCONA, 0x40)
            self._bus.write_byte_data(self._addr, MCP23017Plugin.IOCONB, 0x40)
        else:
            self._bus.write_byte_data(self._addr, MCP23017Plugin.IOCONA, 0x00)
            self._bus.write_byte_data(self._addr, MCP23017Plugin.IOCONB, 0x00)
            
        #GPIO for interruption 
        if self._it!=0:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            for gpio in map(int,Parameters["Mode4"].split(',')):
                Domoticz.Log("Activate Interruption on GPIO  %d"%gpio)
                GPIO.setup(gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                try:
                    GPIO.add_event_detect(gpio, GPIO.FALLING, callback=self.__interruptCall)
                except Exception as e :
                    Domoticz.Error("%s"%str(e))
        else:
            Domoticz.Log("No interruption")
        #Set heartbeat to polling value
        pooling=int(Parameters["Mode5"])
        if pooling==0:
            Domoticz.Log("Disable pooling")
            self._pooling=False
        elif pooling <= 20 :
            Domoticz.Heartbeat(pooling)
        else:
            self._poolingTime=pooling/10

    def __logIt(self,level,msg):
        
        if level not in self._logLevel:
            level='Log'
        if self._it != 0:
            self._itLogger[level]+=[msg]
        else:
            self.myLog(level,msg)

    def myLog(self,level,msg):
        if level=='Debug':
            Domoticz.Debug(msg)
        elif level=='Log':
            Domoticz.Log(msg)
        elif level=='Status':
            Domoticz.Status(msg)
        elif level=='Error':
            Domoticz.Error(msg)
            
    def __interruptCall(self,callback):
        import Domoticz
        self.__logIt('Debug',"Interruption catched")
        for i in range (0,2):
            if self._mask[i] > 0:
                self.updateState(i)

    def __createDevice(self,toAdd):
        Domoticz.Debug("Creating New devices: "+str(toAdd))
        Domoticz.Debug("IO Direction: %s" %self._fullIODir)
        for i in toAdd:
            Domoticz.Device(Name='Input %d'%i, Unit=(i), TypeName="Switch", Subtype=0x49,Switchtype=0, Image=9).Create()
                
    def __initPort(self,port,it=False):
        self.__logIt('Log',"Init port: %d"%port)
        #set selected IO to input
        self._bus.write_byte_data(self._addr, MCP23017Plugin.IODIRA+port, self._mask[port])
        #set Inversion
        self.__logIt('Log',"Set inverion mask for port %d: %s"%(port,bin(self._inversionMask[port])[2:].rjust(8,'0')))
        self._bus.write_byte_data(self._addr, MCP23017Plugin.IOPOLA+port, self._inversionMask[port])
        #set pullup 
        self._bus.write_byte_data(self._addr, MCP23017Plugin.GPPUA+port, self._mask[port])
        if it:
            self.__logIt('Debug',"Enable interrupt for port %d"%port)
            #set It for input ports
            self._bus.write_byte_data(self._addr, MCP23017Plugin.GPINTENA+port, self._mask[port])
        else:
            self._bus.write_byte_data(self._addr, MCP23017Plugin.GPINTENA+port, 0x00)
            
    def updateState(self,port):
        onceMore=False
        try :
            status  = self._bus.read_byte_data(self._addr, MCP23017Plugin.GPIOA+port)&self._mask[port]
        except Exception:
            onceMore=True
        if onceMore:
            try :
                time.sleep(0.8)
                status  = self._bus.read_byte_data(self._addr, MCP23017Plugin.GPIOA+port)&self._mask[port]
            except Exception:
                self.__logIt('Error',"can not get status, read error")
                return
        self.__logIt('Debug','Status for port %d: %s'%(port,bin(status)[2:].rjust(8,'0')))

        #Select only IO that have changed
        changes = self._lastState[port]^status
        i=0
        if changes!=0:
            #workaround for MCP23017 auto reset
            conf=self._bus.read_byte_data(self._addr, MCP23017Plugin.IODIRA+port)
            if(conf!=self._mask[port]):
                self.__logIt('Error',"MCP20317 reset, reconfiguring port %d (%s to %s)"%(port,bin(conf)[2:].rjust(8,'0'),bin(self._mask[port])[2:].rjust(8,'0')))
                self.__initPort(port)
                status  = self._bus.read_byte_data(self._addr, MCP23017Plugin.GPIOA+port)&self._mask[port]
                changes = self._lastState[port]^status
                if changes==0:
                    return
            # -- End Workaround --
            self.__logIt('Log','Update port %d : %s'%(port,bin(changes)[2:].rjust(8,'0')))
            self.__logIt('Log','New values for port %d are %s'%(port,bin(status)[2:].rjust(8,'0')))
        statusStr=bin(status)[2:].rjust(8,'0')[::-1]
        for s in bin(self._mask[port])[2:].rjust(8,'0')[::-1]:
            if s == '1' :
                Devices[(i+1+port*8)].Refresh()
                if str(i+1+port*8) not in self._inTransition:
                    val=statusStr[i]
                    if Devices[(i+1+port*8)].nValue!=int(val):
                        self.__logIt('Debug',"Updating %d"%(i+1+port*8))
                        self.updateUnit((i+1+port*8),int(val))
            i=i+1
        self._lastState[port]=status
        self.__logIt('Debug',"In transition! %s"%(str(self._inTransition)))
                    
    #Update associated switch (idx_cmd)
    def updateUnit(self,unit, nvalue):
        if nvalue==1:
            svalue="On"
        else:
            svalue="Off"
        Devices[unit].Update(nvalue,svalue)
        r_idx = self.findOption(unit,'idx_cmd')
        if self.mqttClient:
            if r_idx:
                msg=json.dumps({'idx':r_idx,'nvalue':nvalue,'svalue':svalue})
                self.mqttClient.publish('domoticz/in',msg,0)
            if self._slaveMode:
                self.mqttPublish(Devices[unit])
                
    def onHeartbeat(self):
        Domoticz.Debug("onHeartbeat called")
        for level in self._logLevel:
            Domoticz.Debug("IT Logs")
            for msg in self._itLogger[level]:
                self.myLog(level,msg)
            self._itLogger[level] = []
            Domoticz.Debug("End of IT Logs")
                
        if self._pooling and (self._poolingTime==0 or self._beat>self._poolingTime):
            #Read status
            if self._mask[0] > 0:
                self.updateState(0)
            if self._mask[1] > 0:
                self.updateState(1)
            self._beat=0
        self._beat+=1
    
        if self.mqttClient is not None:
            try:
              if (self.mqttClient._connection is None) or (not self.mqttClient.isConnected):
                  Domoticz.Debug("Reconnecting")
                  self.mqttClient._open()
              else:
                   self.mqttClient.ping()
            except Exception as e:
              Domoticz.Error(str(e))
            to_delete=[]
            for unit,cpt in self._inTransition.items():
                if int(cpt)>0:
                    self._inTransition[str(unit)]=(int(cpt)-1)
                else:
                    to_delete+=[str(unit)]
            for td in to_delete:
                self._inTransition.pop(td)
              
        return
    
    def onDeviceAdded(self):
        Domoticz.log("Adding device")

    def onStop(self):
        if self._it!=0:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        return

    def onConnect(self,Connection, Status, Description):
        if self.mqttClient is not None:
            self.mqttClient.onConnect(Connection, Status, Description)
        
    def onMessage(self,Connection,Data):
        if self.mqttClient is not None:
            self.mqttClient.onMessage(Connection, Data)

    def findOption(self,unit,option):
        value = None
        for line in Devices[int(unit)].Description.split('\n'):
            if line[:(len(option))] == option:
                value=int(line[(len(option)+1):])
        return value
            
    def onCommand(self,Unit,Command,Level,Hue):
        r_idx=self.findOption(Unit,'idx_cmd')
        ro_idx=self.findOption(Unit,'idx_only_cmd')

        if ro_idx:
            r_idx=ro_idx
        
        if r_idx:
            if not ro_idx:
                msg=json.dumps({'idx':r_idx,'nvalue':Devices[Unit].nValue,'svalue':Devices[Unit].sValue})
                self.mqttClient.publish('domoticz/in',msg,0)
            msg=json.dumps({'idx':r_idx,'command':'switchlight','switchcmd':Command})
            self.mqttClient.publish('domoticz/in',msg,0)
        
            delay=self.findOption(Unit,Command.lower()+'_delay')
            if delay:
                self._inTransition[str(Unit)]=delay
            if Command=="On":
                nval=1
            else:
                nval=0
            Devices[int(Unit)].Update(nval,Command)
        return

    def onNotification(self,Name,Subject,Text,Status,Priority,Sound,ImageFile):
        return

    def onDisconnect(self,Connection):
        if self.mqttClient is not None:
            self.mqttClient.onDisconnect(Connection)

    def onMQTTConnected(self):
      if self.mqttClient is not None:
          self.mqttClient.subscribe([self.base_topic + '/in/#','domoticz/slaves'])
          
    def onMQTTDisconnected(self):
      Domoticz.Debug("onMQTTDisconnected")

    def onMQTTSubscribed(self):
        Domoticz.Debug("onMQTTSubscribed")

    def onMQTTPublish(self, topic, message): # process incoming MQTT statuses
        if self._slaveMode:
            if(topic=="domoticz/slaves"):
                cmd=json.loads(json.dumps(message))
                if (cmd['cmd']=="update_all"):
                    for dev in Devices:
                        self.mqttPublish(Devices[dev])
            elif topic==self.base_topic+'/in/req_info':
                dev=json.loads(json.dumps(message))
                for unit in Devices:
                    if Devices[unit].ID==dev['idx']:
                        device=Devices[unit]
                        msg=json.dumps({"idx":device.ID,"status":"Ok","svalue":device.sValue,"nvalue":device.nValue,"name":device.Name,"type":device.Type,"subtype":device.SubType,"switchtype":device.SwitchType,"description":device.Description,'image':device.Image,"action":"read-only",'options':device.Options})
                        self.mqttClient.publish(self.base_topic+'/out/req_info',str(msg),0)
                        return
                msg=json.dumps({"idx":dev['idx'],"status":"Not Found"})
                self.mqttClient.publish(self.base_topic+'/out/req_info',str(msg),0)

    def mqttPublish(self,device):
        if self.mqttClient is not None:
            msg=json.dumps({"idx":device.ID,"svalue":device.sValue,"nvalue":device.nValue})
            self.mqttClient.publish(self.base_topic+'/out',str(msg),0)
        
global _plugin
_plugin = MCP23017Plugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onConnect(Connection, Status, Description):
    global _plugin
    _plugin.onConnect(Connection, Status, Description)

def onMessage(Connection, Data):
    global _plugin
    _plugin.onMessage(Connection, Data)

def onCommand(Unit, Command, Level, Hue):
    global _plugin
    _plugin.onCommand(Unit, Command, Level, Hue)

def onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile):
    global _plugin
    _plugin.onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile)

def onDisconnect(Connection):
    global _plugin
    _plugin.onDisconnect(Connection)

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

def  onDeviceAdded():
    global _plugin
    _plugin.onDeviceAdded()
    # Generic helper functions

def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug( "'" + x + "':'" + str(Parameters[x]) + "'")
            Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))
    return


