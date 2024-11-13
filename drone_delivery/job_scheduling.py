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
    box_ids = [box.box_id for inventory in state.inventories for box in inventory]

    # return list of possible actions
    for drone_id in state.drones.keys():
        # get combinations of pairs of locations unless pl == pharmacy and fl == house
        for start_pos, end_pos in combinations(locations, 2):
            # can't move from house to house
            if start_pos.type == 'H' and end_pos.type == 'H':
                # move from pickup to dropoff location
                actions.append(Move(drone_id, start_pos, end_pos))
                # move the other way
                actions.append(Move(drone_id, end_pos, start_pos))
        for box_id in box_ids:
            actions.append(Pickup(drone_id, box_id))
            actions.append(Dropoff(drone_id, box_id))

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
        return all(inventory == [] for inventory in state.inventories) and all(drone.status == None for drone in state.drones)


class MaxHeuristic():
    """
    total_distance =sum(manhatten_distance(obj.start_pos, obj.end_pos) for inventory in inventories for obj in inventory)
	objects_undelivered = sum(len(inventory) for inventory in inventories.values())
	Return distance_heuristic + objects_undelivered
    """
    def __init__(self, state):
        return 0

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
            # actions_added = 0
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
                        
                        #actions_added += 1
            #print('actions_added '+str(actions_added))
        return None # No plan was found

#example problem


inventories = {}
#importing house and pharmacy locations
with open('locations.csv', newline='') as csvfile:
    pharmacies, houses = [],[]
    locations =  csv.reader(csvfile, delimiter=' ', quotechar='|')
    location_id = 0
    for row in locations:
        cols = row.split(",")
        if row[0] == "H": 
            houses.append(House(location_id, (row[1],row[2],row[3])))
        elif row[0] == "P":
            pharmacies.append(Pharmacy(location_id, (row[1],row[2],row[3])))
            inventories[(row[1],row[2],row[3])] = []
            pass

        location_id += 1

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
inventory1 = {
    # box_id or order_id, box object
    0: box1, 
    1: box2, 
    2: box3
    }
inventory2 = {
    3: box4, 
    4: box5
    }


inventories = {
    #pharmacy_id, pharmacy.inventory
    0: inventory1,
    1: inventory2
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
