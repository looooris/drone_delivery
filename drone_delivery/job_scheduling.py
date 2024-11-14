from queue import PriorityQueue
import csv
from actions import Move, Pickup, Dropoff, generate_actions
from drone_delivery.objects import Drone, House, Pharmacy, Box
from itertools import combinations

# runs once at beginning of program
def generate_actions(state, locations):
    actions = []
    # list of actions
    # pick up: (for all drones in state, add to list of actions)
    # drop off: (for all drones in state, add to list of actions)
    # fly to: (for all drones in state, add to list of actions)

    #iterate through drones and inventories
    #drone: id, position, status
    #inventory: [Box(pl, dl), Box(pl, dl)]
    #merge all inventory lists
    boxes = [box for inventory in state.inventories for box in inventory]

    # return list of possible actions
    for drone_id in state.drones.keys():
        # get combinations of pairs of locations unless pl == pharmacy and fl == house
        for start_pos, end_pos in combinations(locations, 2):
            # can't move from house to house
            if not (start_pos.type == 'H' and end_pos.type == 'H'):
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
        return all(len(inventory) == 0 for inventory in state.inventories) and all(drone.status == None for drone in state.drones)


class MaxHeuristic():
    """
    total_distance =sum(manhatten_distance(obj.start_pos, obj.end_pos) for inventory in inventories for obj in inventory)
	objects_undelivered = sum(len(inventory) for inventory in inventories.values())
	Return distance_heuristic + objects_undelivered
    """
    def __init__(self, problem):
        self.problem = problem
        self.locations = problem.locations
        return 0
    
    def h(self, state):
        # modify heuristic to calculate more accurately
        # double distance_left? most drones have to do at least 1 trip to the house and 1 trip back to the pharmacy
        # factor in actions
        distance_left = sum(self.manhatten_distance(box.pickup_pos, box.dropoff_pos) for inventory in state.inventories for box in inventory)
        boxes_undelivered = sum(len(inventory) for inventory in state.inventories)
        
        heuristic = distance_left + boxes_undelivered
        return heuristic

    
    def manhatten_distance(start_pos, end_pos):
        pass

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
        locations = problem.locations()
        closed = []
        frontier = PriorityQueue()

        # calculate the initial heuristic value for the first state and add them to frontier
        h = self.h(problem.state)
        frontier.put((h, Node(problem.initial_state)))

        # execute while there are still nodes in the queue
        while not frontier.empty():

            # retrieve the node with the smallest f value
            f, node = frontier.get()
            
            # assign the node's state to variable s
            s = node.state

            # if at goal, return current sequence of steps by backtracking
            if problem.at_goal(s):
                #print("finishing solving")
                return path_actions(node)
            
            # loop through all the possible actions to check which ones can be applied to the current state
            for action in problem.get_actions():

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
                        # now that we've evaluated the new state, put it in closed list to prevent it being reevaluated
                        closed.append((f, Node(s1, node, action, new_path_cost)))
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
    # done_id, pos (x, y, z), status (object or none)
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

# parse object data from csv file
# uncomment the line below to run
#drones, inventories, locations = parse_ros_objects('locations.csv')


initial_state = (drones, inventories)
actions = generate_actions(initial_state, locations)

pb = MultiAgentJobScheduling(initial_state, actions, locations)
planner = AStarPlanner(problem=pb)
plan = planner.solve()

resulting_state = initial_state
# curate list of tasks for each drone
for action in plan:
    drone_id = action.drone_id
    box = resulting_state.drones[drone_id].status
    
    resulting_state = action.apply(resulting_state)
    next_pos = resulting_state.drones[drone_id].pos

    # returns (move name, next pos, box object)
    drones[drone_id].plan.append((action.name, resulting_state.drones[drone_id].pos, box))

# extract the plans for each drone
drone_action_plans = [(drone.drone_id, drone.plan) for drone in drones]