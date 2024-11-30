from queue import PriorityQueue
import csv

#ROS2 IMPORTS
from drone_delivery.dd_actions import Move, Pickup, Dropoff
from drone_delivery.objects import Drone, House, Pharmacy, Box

#LOCAL PYTHON IMPORTS (COMMENT ABOVE AND UNCOMMENT BELOW)
#from objects import Drone, House, Pharmacy, Box
#from dd_actions import Move, Pickup, Dropoff
from itertools import combinations
import random 

# runs once at beginning of program
def generate_actions(state, locations):
    actions = []
    # list of actions
    # pick up: (for all drones in state, add to list of actions)
    # drop off: (for all drones in state, add to list of actions)
    # fly to: (for all drones in state, add to list of actions)

    #iterate through drones and inventories
    #state = (drones, inventories)

    #drones (dictionary) = { drone_id: Drone(0, (1, 1, 1))},
    #location_id (integer)

    #inventories (dictionary) = {location_pos : inventory}
    #location_pos (tuple) = (x, y, z)
    #inventory (set) = {Box(pl, dl), Box(pl, dl)}

    #merge all inventory sets
    boxes = [box for inventory in state[1].values() for box in inventory]

    # return list of possible actions
    for drone_id in state[0].keys():
        # get combinations of pairs of locations unless pl == pharmacy and fl == house
        for start_pos, end_pos in combinations(locations, 2):
            # can't move from house to house
            if not (locations[start_pos].type == 'H' and locations[end_pos].type == 'H'):
                # move from pickup to dropoff location
                actions.append(Move(drone_id, start_pos, end_pos))
                # move the other way
                actions.append(Move(drone_id, end_pos, start_pos))
        for box in boxes:
            actions.append(Pickup(drone_id, box))
            actions.append(Dropoff(drone_id, box))

    return actions

class MultiAgentJobScheduling():
    # doesn't need a goal because empty inventory is goal
    def __init__(self, initial_state, actions, locations):
        # drones, inventories (each one belongs to a pharmacy)
        # locations: ids and locations of pharmacies and houses
        self.initial_state = initial_state #import csv file
        self.locations = locations
        self.actions = actions

    def get_actions(self):
        return self.actions

    def at_goal(self, state):
        # return if inventories are all empty and drone statuses are None
        return all(len(inventory) == 0 for inventory in state[1].values()) and all(drone.status == None for drone in state[0].values())


class MaxHeuristic():
    """
    total_distance =sum(manhatten_distance(obj.start_pos, obj.end_pos) for inventory in inventories for obj in inventory)
	objects_undelivered = sum(len(inventory) for inventory in inventories.values())
	Return distance_heuristic + objects_undelivered
    """
    def __init__(self):
        pass

    def __call__(self, state):
        return self.h(state)
    
    def h(self, state):
        # modify heuristic to calculate more accurately
        # double distance_left? most drones have to do at least 1 trip to the house and 1 trip back to the pharmacy
        # factor in box priorities too
        total_distance = 0
        #print('inventory length:',len(state[1]))
        """
        for id, inventory in state[1].items():
            print('inventory id:',id)
            for box in inventory:
                total_distance += self.manhatten_distance(box.pickup_pos, box.dropoff_pos)

        """
        
        box_distance_left = sum(self.manhatten_distance(box.pickup_pos, box.dropoff_pos) for inventory in state[1].values() for box in inventory)
        #boxes_undelivered = sum(len(inventory) for inventory in state[1].values())
        #box_distance_left = 0
        
        #heuristic = box_distance_left + boxes_undelivered
        heuristic = box_distance_left
        return heuristic

    
    def manhatten_distance(self, start_pos, end_pos):
        x1, y1, z1 = start_pos
        x2, y2, z2 = end_pos

        return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)

class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __repr__(self): return '<{}>'.format(self.state)
    def __len__(self): return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): return self.path_cost < other.path_cost

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []
    return path_actions(node.parent) + [node.action]


class AStarPlanner():
    def __init__(self, heuristic=MaxHeuristic()):
        self.h = heuristic
        pass

    # run multiple times during runtime to constantly have the most optimal route for each drone
    def solve(self, problem):
        locations = problem.locations
        closed = []
        frontier = PriorityQueue()

        # calculate the initial heuristic value for the first state and add them to frontier
        h = self.h(problem.initial_state)
        frontier.put((h, Node(problem.initial_state)))

        # execute while there are still nodes in the queue
        while not frontier.empty():
            #print('nodes in frontier:',len(frontier.queue))
            """
            print('Possible moves:')
            for _, n in frontier.queue:
                if n.action:
                    n.action.describe_move(locations)
            print()
            """
            

            # retrieve the node with the smallest f value
            f, node = frontier.get()
            
            
            # assign the node's state to variable s
            s = node.state

            # if at goal, return current sequence of steps by backtracking
            if problem.at_goal(s):
                print("finishing solving")
                return path_actions(node)
            
            # loop through all the possible actions to check which ones can be applied to the current state
            for action in problem.get_actions():
                """
                print(f'*** describing {action.name} ***')
                action.describe_move(locations)
                print('*** describing the state ***')
                action.describe_state(s, locations)
                print('is applicable? ', action.applicable(s, locations))
                print()
                """
                
                # calculate next state only if the action leads to a valid state that
                # satisfies both positive and negative preconditions
                if action.applicable(s, locations):
                    s1 = action.apply(s)

                    # evaluate the generated state if it hasn't been explored yet
                    if s1 not in closed:

                        # edges of graph are unweighted, so the step cost between states is 1
                        new_path_cost = node.path_cost+1

                        # calculate the new heuristic value at the new state
                        h = self.h(s1)
                        f = h + new_path_cost
                        frontier.put((f, Node(s1, node, action, new_path_cost)))
                        # now that we've evaluated the new state, put it in closed list to prevent it being reevaluated
                        closed.append(s1)
        return None # No plan was found

#example problem



#importing house and pharmacy locations
def parse_ros_objects(csv_file):
    # drone objeccts
    drones = {}
    # set of boxes for inventory at each pharmacy
    inventories = {}
    boxes = []
    # manhole locations
    locations = {}

    # object pointers for object ids
    location_id = 0
    box_id = 0
    drone_id = 0

    with open(csv_file, newline='') as csvfile:
        locations =  csv.reader(csvfile, delimiter=' ', quotechar='|')
        
        for row in locations:
            col = row.split(",")
            object_name = col[0]
            pos = (col[1], col[2], col[3])

            # ex: 'H,16,15,0'
            if object_name == 'H': 
                house = House(location_id, pos)
                locations[pos] = house
                location_id += 1
                
            # ex: 'P,-25,-25,0'
            elif object_name == 'P':
                pharmacy = Pharmacy(location_id, pos)
                locations[pos] = pharmacy
                inventories[pos] = {}
                location_id += 1

            # ex: 'B,-25,-25,0,16,15,0,None'
            elif object_name == 'B':
                dropoff_pos = (col[4], col[5], col[6])
                priority = col[7]
                box = Box(box_id, pos, dropoff_pos, priority)
                boxes.append(box)
                box_id += 1

            # ex: 'D,0,0,0,None'
            elif object_name == 'D':
                status = col[4]
                drone = Drone(drone_id, pos, status)
                drones[drone_id] = drone
                drone_id += 1
            else:
                pass

    for box in boxes:
        if inventories.get(box.pos):
            inventories[box.pos].add(box)

    return drones, inventories, locations


def generate_plan(drones, inventories, locations):
    initial_state = (drones, inventories)
    actions = generate_actions(initial_state, locations)

    pb = MultiAgentJobScheduling(initial_state, actions, locations)
    planner = AStarPlanner()
    plan = planner.solve(problem=pb)

    resulting_state = initial_state
    if plan is not None:
        drone_action_plans = {}
        # curate list of tasks for each drone
        for action in plan:
            drone_id = action.drone_id
            #print('drone id: '+str(drone_id))

            current_state = resulting_state
            current_box_status = current_state[0][drone_id].status
            
            resulting_state = action.apply(resulting_state)
            resulting_box_status = resulting_state[0][drone_id].status
            resulting_drone_pos = resulting_state[0][drone_id].pos
            resulting_pos_type = locations[resulting_drone_pos].type

            box_status = None
            if current_box_status:
                box_status = current_box_status
            elif resulting_box_status:
                box_status = resulting_box_status

            # returns (move name, next pos, box object)
            step = (action.name, resulting_pos_type, resulting_drone_pos, box_status)
            drone_plan = drone_action_plans.get(drone_id)
            if drone_plan:
                drone_plan.append(step)
            else:
                drone_action_plans[drone_id] = [step]

        # extract the plans for each drone
        #drone_action_plans = [(drone.drone_id, drone.plan) for drone in drones.values()]
        return drone_action_plans
    
    return None


def print_plans(plans):
    print('list of drone action plans:\n')
    for drone_id, plan in plans.items():
        print('drone_id: '+str(drone_id))
        for step in plan:
            action_name, resulting_pos_type, resulting_pos, box_status = step
            if box_status: box_status = box_status.box_id
            print(f'action name: {action_name}, next_pos ({resulting_pos_type}): {resulting_pos}, box status: {box_status}')
        print()

def example_hardcoded_params():
    # example with hardcoded ids and coordinates
    l1 = (2, 3, 3) #pharmacy
    l2 = (6, 3, 1) #Pharmacy
    l3 = (4, 2, 7) #house
    l4 = (2, 0, 1) #House

    # pickup location, dropoff location, priority status (default is none)
    box1 = Box(0, l1, l3)
    box2 = Box(1, l1, l3, 'urgent')
    box3 = Box(2, l1, l4)
    box4 = Box(3, l2, l3)
    box5 = Box(4, l2, l4)

    drones = {
        # done_id, pos (x, y, z), status (box object or none)
        0: Drone(0, (1, 1, 1)),
        1: Drone(1, (1, 2, 2), box1),
        2: Drone(2, (1, 4, 3), box2)
        }
                
    # each location (pharmacy or house) will have a unique id
    # box_id or order_id, box object
    inventory1 = {box1, box2, box3}
    inventory2 = {box4, box5}


    inventories = {
        #pharmacy_id, pharmacy.inventory
        l1: inventory1,
        l2: inventory2
    }

    pharmacy1 = Pharmacy(0, l1)
    pharmacy2 = Pharmacy(1, l2)
    house1 = House(2, l3)
    house2 = House(3, l4)


    locations = {
        # (x, y, z), location object
        l1: pharmacy1, 
        l2: pharmacy2, 
        l3: house1, 
        l4: house2
                }
    return drones, inventories, locations


def randomise_world(number_of_drones):
    # example with hardcoded ids and coordinates
   
    l1 = (-6.49895, -40.7371, 0) # Pharmacy
    l2 = (-45.65, 38.58, 0) # Pharmacy
    l3 = (30.55, -26.11, 0) # House
    l4 = (-60.65, -27.63, 0) # House
    l4 = (16.78, 40.43, 0) # House
    l5 = (-60.92, 17.92, 0) # Hose
    l6 = (54.04, 44.62, 0) # House

    pharmacyList = [l1, l2]
    houseList = [l3, l4, l5, l6]


    # box id, pickup location, dropoff location, priority status (default is none)
    if number_of_drones == 2:
        box1 = Box(0, random.choice(pharmacyList), random.choice(houseList))
        box2 = Box(1, random.choice(pharmacyList), random.choice(houseList))
        box3 = Box(2, random.choice(pharmacyList), random.choice(houseList), 'urgent')
        box4 = Box(3, random.choice(pharmacyList), random.choice(houseList))
        box5 = Box(4, random.choice(pharmacyList), random.choice(houseList))
        
        boxes = [box1, box2, box3, box4, box5]
    
    else:
        box1 = Box(0, random.choice(pharmacyList), random.choice(houseList))
        box2 = Box(1, random.choice(pharmacyList), random.choice(houseList), 'urgent')
        box3 = Box(2, random.choice(pharmacyList), random.choice(houseList))

        boxes = [box1, box2, box3]

    inventory1 = set()
    inventory2 = set()

    for box in boxes:
        if (box.returnPickup() == l1):
            inventory1.add(box)
        else:
            inventory2.add(box)

    drones = {}
    for z in range(number_of_drones):
        drones[z] = Drone(z, (z, z, 0))
        
    # each location (pharmacy or house) will have a unique id
    # box_id or order_id, box object
    #inventory is a set

    inventories = {
        #pharmacy_id, pharmacy.inventory
        l1: inventory1,
        l2: inventory2
    }

    pharmacy1 = Pharmacy(0, l1)
    pharmacy2 = Pharmacy(1, l2)
    house1 = House(l3)
    house2 = House(l4)
    house3 = House(l5)
    house4 = House(l6)

    locations = {
        # (x, y, z), location object
        l1: pharmacy1, 
        l2: pharmacy2, 
        l3: house1, 
        l4: house2,
        l5: house3,
        l6: house4
                }

    plan = generate_plan(drones, inventories, locations)
    returnPlans = [None] * number_of_drones

    for singleItem in range(len(plan)):
        #returnPlans[singleItem] = [[singleItem, singleItem, singleItem, 0, 1]]
        returnPlans[singleItem] = []
        for singleAction in plan[singleItem]:
            if(singleAction[0] == "move"):
                if singleAction[1] == 'H':
                    goal = 0
                else:
                    goal = 1

                returnPlans[singleItem].append([singleAction[2][0], singleAction[2][1], singleAction[2][2], goal])
        returnPlans[singleItem].append([singleItem, singleItem, 0, 1])

    # avoid sending robots to the same place initially
    if len(plan) > 1 and len(returnPlans[0]) > 2:
        if returnPlans[0][0] == returnPlans[1][0]:
            if returnPlans[0][2] == returnPlans[1][0]:
                temp = returnPlans[1][0]
                temptwo = returnPlans[1][1]
                returnPlans[1][0] = returnPlans[1][2]
                returnPlans[1][1] = returnPlans[1][3]
                returnPlans[1][2] = temp
                returnPlans[1][3] = temptwo
            else:
                temp = returnPlans[0][0]
                temptwo = returnPlans[0][1]
                returnPlans[0][0] = returnPlans[0][2]
                returnPlans[0][1] = returnPlans[0][3]
                returnPlans[0][2] = temp
                returnPlans[0][3] = temptwo    
    #print(returnPlans)            
    return returnPlans


# example reading from csv file. uncomment the line below to get params
#drones, inventories, locations = parse_ros_objects('locations.csv')

# example with hardcoded values. uncomment the line below to get params
#drones, inventories, locations = example_hardcoded_params()

# example with hardcoded values. uncomment the line below to get params
#randomise_world(2)
#randomise_world(1)

# get list of tasks for each drone
# drone_action_plans = generate_plan(drones, inventories, locations)
# print_plans(drone_action_plans)

"""
parses input into drones, inventories, and locations and feeds these into a planner to create an optimal plan for each drone

input types:
Drone: drone id (string), current drone position (int tuple -> (x, y, z)), urgency? (int)
Location: type (string -> P or H), position (int tuple -> (x, y, z))
Box: pickup position (int tuple -> (x, y, z)), dropoff position (int tuple -> (x, y, z)), current position (int tuple -> (x, y, z)), urgency? (int)

return types:
actions (dict) = {
    drone_id (string) : actions (list) [
        action data (set) = ( action name (string), P or H (string), resulting_drone_pos (int tuple -> (x, y, z)), box_status (Box object or None) )
    ]
}

notes:

things to randomize:
drone initial positions: (0, 0, 0) or (1, 1, 0)
box pickup and dropoff positions

"""