import enum
from geometry import *
from collections import *
#Data related to Segment
class Segment_Type(enum.Enum):
    Safe = 0
    Collision = 1
    Wait = 2

#Data related to Waypoint 
class Waypt_Status(enum.Enum):
    #Intermediate Goal
    Inter = 0
    #Final Goal
    Goal = 1
    #Wait to avoid collision
    Wait = 2

class Waypoint:
    def __init__(self, x, y, time, status):
        self.pos = Point(x,y)
        self.time_to_reach = time
        self.status = status 

    def __str__(self):
        return f"Pose {self.pos} Time {self.time_to_reach} Status {self.status}"

class Status(enum.Enum):
    MoveToGoal = 0
    ReachGoal = 1 
    Loading = 2
    Loaded = 3
    Unloading = 4
    Unloaded = 5
    Planning = 6
    WaitForInboundTask = 7
    WaitForOutboundTask = 8
    WaitForInboundStation = 9
    Parked = 10
    InOutboundStation = 11
    Chargig = 12
    Queue = 13
    Charged = 14
    WaitForOutboundStation = 15
    WaitForLoadOutbound = 16

class Task(enum.Enum):
    InboundStation = 0
    OutboundStation = 1
    Inbound = 2
    Outbound = 3
    ParkStation = 4
    WaitForTask = 5
    ChargingStation = 6
    # For the load balancing outbound task
    Outbound_Task = 7

class Charge_Station_Mode(enum.Enum):
    Occupied = 0
    Free = 1

class Mode_Of_Operation(enum.Enum):
    Charge_Mode = 0  # the robot is charging
    Work_Mode = 1   # the robot is working
    Queue_Mode = 2  # the robot is in queue
    Off_Mode = 3    # the robot is off
    On_Mode = 4     # the robot is on, but what is the difference between work and on?



#Data Stored in BlackBoard
class Data(enum.Enum):
    #robot id Data: int
    RID = 0
    #Robot pos Data: Point(x,y)
    Pos = 1
    #Robot orientation Data: float
    Theta = 2
    #Robot Velocity Data: Point(Vel_x, Vel_y)
    Vel = 3
    #Robot Goal Data: Point(Goal_x, Goal_y)
    Goal = 4
    #Robot Routine Data: T{MoveToGoal, ReachGoal, }
    RobotStatus = 5
    #Robot TaskStatus
    TaskStatus = 6 
    #Robot Path Data: [Point(x1, y1), Point(x2, y2),... Point(xg, yg)]
    Path = 7
    #Robot Trajectory Segment Data: [[Point(x1,y1), tr+tau, T{Safe, Collision, Wait}],[],...[]]
    Segment = 8
    #Robot Controller profile Data:[[Point(pos_x,pos_y), Point(vel_x,vel_y), float] ,[],...]
    Control_Profile = 9
    #Robot Inbound_Station Data: [[ID1, Point(x1, y1)], [ID2, Point(x2, y2)],...]
    Inbound = 10
    #Robot Outbound_Station Data: [[ID1, Point(x1,y1)], [ID2, Point(x2,y2)],...]
    Outbound = 11
    #Robot Pick Data: [[TaskID_1, Point(x1,y1)], [TaskID_2, Point(x2,y2)],...]
    Pick = 12
    #Robot Drop Data: [[TaskID_1, Point(x1,y1)], [TaskID_2, Point(x2,y2)],...]
    Drop = 13
    #Robot Inbound Station: Inbound_ID:
    Inbound_Station = 14
    #Assign Robot Inbound Task: bool
    Assign_Inbound = 15
    #Assigne Robot Outbound Task: bool
    Assign_Outbound = 16
    #Assign Inbound station
    Assign_Inbound_Station = 17
    #Initial Pose [[RobotID_1, Point(x1,y1)], [Robot_id_2, Point(x2,y2)],...]
    Initial_Pose = 18
    #Robot partition, Which contain global id of each robot
    Robot_Member = 19
    #Layout Polygon [[Point(x1,y1), Point(x2,y2), Point(x3,y3), Point(x4,y4)],...]
    Polygon = 20
    #Layout Points [[Point(x,y)]]
    Node_Point = 21
    #Layout Lines [[Point(x1,y1), Point(x2,y2)]]
    Edge_Line = 22
    #Simulation Start
    Start = 23
    #Task [[ID1, Point(x1,y1)], [ID2, Point(x2,y2)],...] 
    Task = 24
    #Task ID 
    Task_ID = 25
    #Color 
    Color = 26
    #Reservation [[RID, (x,y)],[RID, (x,y)]] 
    Reservation = 27
    #Unreserve [Point(x1,y1), Point(x2,y2)]
    Unreserve = 28
    #HighPriorityrobot [[RID_1, Point(x1,y1)], [RID_2, Point(x2,y2)], ...]
    High_Priority_robot = 29
    #Waitspot
    Waitspot = 30
    #Current Task [[RID, task_id], [RID, task_id]]
    Current_Task = 31
    #Number of Inbound Task
    Number_of_Inbound_Task = 32
    #Number of Outbound Task
    Number_of_Outbound_Task = 33
    #Total Number of Inbound Done
    Inbound_Completed = 34
    #Total Number of Outbound Done
    Outbound_Completed = 35
    #Total Time
    Total_Time = 36
    #Inbound Station State
    Inbound_Station_State = 37
    #Robot Inbound Station
    Robot_Inbound_Station = 38
    #Assign Task bool - True if Task Manager assign task
    Assign_Task = 39
    #Assigned Task
    Assigned_Task = 40
        # Robot Charge Data: float
    Charge_level = 41
    # Charging station in the layout
    Charging_station = 42
    #Global Address : int 
    GID = 43
    #battery : float
    Battery = 44
    #Robot Mode : Enum(Mode_Of_Operation)
    Battery_Mode = 45
    #Charge Station Status
    Charge_Station_Mode = 46
    #Charge Station Robot 
    Charge_Station_Robot = 47
    #Charge Time : int 
    Charge_Time = 48
    #Charge Robot Info : [Charge_Station_ID (int), Queue_ID(int), Critical(bool)]
    Charge_Robot_Info = 49
    #Parking Station: [Point(x,y), ID(int)]
    Parking_Station = 50
    #battery_level : float when enetering the charging station
    CS_Battery_level = 51
    # Robot Full Path
    Full_Path = 52
    # Robot Outbound Task
    Load_Outbound = 53
    # Robot assign Load Outbound Task
    Assign_Load_Outbound = 54
    #Edge: ['id' , 'capacity' , [node[0] , node[1] ,..., , node[-1]]
    # Edge = namedtuple('Edge', ['id', 'capacity', 'location'])
    
#Events Stored in BlackBoard
class Event(enum.Enum):
    #Assign robot initial pose
    Initialize_Pose = 0
    #Robot Reached assigned Goal [Trigger for getting new goal]
    Goal_Reached = 1
    #Robot got new goal [Response of Goal Reached]
    Got_New_Goal = 2
    #Robot Moved to new location 
    Moved = 3
    #Local planner made local plan
    Got_LocalPlan = 4
    #Robot got time profile segment
    Got_Segment = 5
    #Robot controller waypoints
    Debug_Control_Profile = 6
    #Join Into Robot partition
    Join_Robot = 7
    #Leave from Robot partition
    Leave_Robot = 8
    #Start Simulation
    Start = 9
    #Initialize Layout
    Initialize_Layout = 10    