# file robotSimulator.py simulates a robot in an arena
import sys
sys.path.append('/home/ryan/pyckbot/hrb')

import rclpy
from rclpy.node import Node
from pub_sub import MinimalPublisher

from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim, RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import (
    WAYPOINT_HOST, WAYPOINT_MSG_PORT, APRIL_DATA_PORT
    )
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis

class MoveForward(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.move(step)
      yield self.forDuration(dt)

class Turn(Plan):
  """
  Plan simulates robot turning over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Angle to turn [rad]
    self.ang = 0.1
    # Duration of travel [sec]
    self.dur = 3.0
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute rotation step
    dt = self.dur / float(self.N)
    step = self.ang / float(self.N)
    for k in range(self.N):
      s.turn(step)
      yield self.forDuration(dt)


class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.

     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """
  def __init__(self,wphAddr=WAYPOINT_HOST,wphPort=WAYPOINT_MSG_PORT,*arg,**kw):
    """
    Initialize the simulator
    """
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      )
    self.srvAddr = (wphAddr, wphPort)
    # ADD pre-startup initialization here, if you need it

  def onStart( self ):
    """
    Sets up the JoyApp and configures the simulation
    """
    ### DO NOT MODIFY ------------------------------------------
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server="141.213.30.33",port=self.srvAddr[1])
    self.sensor.start()
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now
    ### MODIFY FROM HERE ------------------------------------------
    self.robSim = SimpleRobotSim(fn=None)
    self.moveP = MoveForward(self,self.robSim)
    self.turnP = Turn(self,self.robSim)

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )

  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    #### DO NOT MODIFY --- it WILL break the simulator
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    print(msg.encode("ascii"))
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg.encode("ascii"), (self.srvAddr[0],APRIL_DATA_PORT))

  def on_K_UP(self,evt):
    if self.moveP.isRunning(): return
    self.moveP.dist = 100.0
    self.moveP.start()
    return progress("(say) Move forward")

  def on_K_DOWN(self,evt):
    if self.moveP.isRunning(): return
    self.moveP.dist = -100.0
    self.moveP.start()
    return progress("(say) Move back")
  
  def on_K_LEFT(self,evt):
    if self.turnP.isRunning(): return
    self.turnP.ang = 0.5
    self.turnP.start()
    return progress("(say) Turn left")

  def on_K_RIGHT(self,evt):
    if self.turnP.isRunning(): return
    self.turnP.ang = -0.5
    self.turnP.start()
    return progress("(say) Turn right")

  def onEvent( self, evt ):
    #### DO NOT MODIFY --------------------------------------------
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame():
      # self.emitTagMessage()
      ts,w = self.sensor.lastWaypoints
      rclpy.init()

      mpub = MinimalPublisher()
      mpub.wpoints = w

      rclpy.spin_once(mpub)

      mpub.destroy_node()
      rclpy.shutdown()

    return JoyApp.onEvent(self,evt)

if __name__=="__main__":
  from sys import argv
  print("""
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer at ip and port given on commandline

  USAGE:
    %s
        Connect to default host and port
    %s <host>
        Connect to specified host on default port
    %s <host> <port>
        Connect to specified host on specified port
  """ % ((argv[0],)*3))
  import sys
  cfg = {'windowSize' : [160,120]}
  if len(argv)>2:
      app=RobotSimulatorApp(wphAddr=argv[1],wphPort=int(argv[2]),cfg=cfg)
  elif len(argv)==2:
      app=RobotSimulatorApp(wphAddr=argv[1],cfg=cfg)
  else:
      app=RobotSimulatorApp(cfg=cfg)
  app.run()
