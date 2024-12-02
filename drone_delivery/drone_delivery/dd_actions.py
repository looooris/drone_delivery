#ROS2 IMPORTS
from drone_delivery.objects import Drone, House, Pharmacy, Box

#LOCAL PYTHON IMPORTS (COMMENT ABOVE AND UNCOMMENT BELOW)
#from objects import Drone, House, Pharmacy, Box


def clone_inventory(inventory):
    inventory_clone = {box for box in inventory}
    return inventory_clone


class Action(object):
    def __init__(self, name, drone_id):
        # ex: 'pick up'
        self.name = name
        self.drone_id = drone_id

    def applicable(self, state):
        # check if action is valid
        pass

    def clone(self, state):
        # copy state
        # write copy code here
        # drones, inventories
        # drone = Drone(id, pos (x, y, z), status (object or none))
        drones_clone = {drone_id : drone.clone() for drone_id, drone in state[0].items()}
        inventories_clone = {location_pos : clone_inventory(inventory) for location_pos, inventory in state[1].items()}
        return (drones_clone, inventories_clone)

    def apply(self, state):
        pass


class Pickup(Action):
    def __init__(self, drone_id, box):
        super().__init__(name='pickup', drone_id=drone_id)
        self.drone_id = drone_id
        self.box = box
        self.pharmacy_id = box.pickup_pos

    def applicable(self, state, locations=None):
        # return true if drone has no object AND currently at a pharmacy with inventory
        inventory = state[1].get(self.pharmacy_id)
        drone = state[0][self.drone_id]
        return not state[0][self.drone_id].has_object() and drone.pos == self.pharmacy_id and inventory and len(inventory) > 0

    def apply(self, state):
        resulting_state = super(Pickup, self).clone(state)
        
        # remove box from inventory
        resulting_state[1][self.pharmacy_id].discard(self.box)

        # assign box to drone
        resulting_state[0][self.drone_id].status = self.box

        return resulting_state

    def describe_move(self, locations):
        print(f'drone {self.drone_id} picks up box {self.box.box_id} at {locations[self.box.pickup_pos].type}: {self.box.pickup_pos}')

    def describe_state(self, state, locations):
        drone = state[0][self.drone_id]
        print(f'current drone location: {drone.pos}')
        print(f'current drone status: {drone.status}')

class Dropoff(Action):
    def __init__(self, drone_id, box):
        super().__init__(name='dropoff', drone_id=drone_id)
        self.box = box

    def applicable(self, state, locations=None):
        # return true if drone has box AND drone is at box's dropoff location
        drone = state[0][self.drone_id]
        return drone.has_object() and drone.pos == self.box.dropoff_pos

    def apply(self, state):
        resulting_state = super(Dropoff, self).clone(state)

        # reset drone status to None
        resulting_state[0][self.drone_id].status = None

        return resulting_state

    def describe_move(self, locations):
        print(f'drone {self.drone_id} drops off box {self.box.box_id} at {locations[self.box.dropoff_pos].type}: {self.box.dropoff_pos}')

    def describe_state(self, state, locations):
        drone = state[0][self.drone_id]
        print(f'current drone location: {drone.pos}')
        print(f'current drone status: {drone.status}')

class Move(Action):
    def __init__(self, drone_id, takeoff_location, landing_location):
        super().__init__(name='move', drone_id=drone_id)
        self.drone_id = drone_id
        self.takeoff_location = takeoff_location
        self.landing_location = landing_location

    def applicable(self, state, locations=None):
        # drone at takeoff location
        # drone start pos does not equal end pos - already taken care of in action generation
        # start pos location and end location are not both houses - already taken care of in action generation
        drone = state[0][self.drone_id]
        box = drone.status

        # if drone carrying box and box dropoff pos == landing position
        if drone.has_object() and box.dropoff_pos == self.landing_location:
            return True
        
        # drone not carrying box AND landing pharmacy still has inventory
        if not drone.has_object() and locations[self.landing_location].type == 'P':
            # get the pharmacy id of the box's dropoff location
            inventory = state[1].get(self.landing_location)
            if inventory and len(inventory) > 0:
                return True
       
        return False

    def apply(self, state):
        resulting_state = super(Move, self).clone(state)

         # set drone pos to end pos
        resulting_state[0][self.drone_id].pos = self.landing_location

        return resulting_state

    def describe_move(self, locations):
        # outputs the drone's imminent journey
        print(f'drone {self.drone_id} moves from {locations[self.takeoff_location].type}: {self.takeoff_location} to {locations[self.landing_location].type}: {self.landing_location}')

    def describe_state(self, state, locations):
        # outputs the current state of the drone
        drone = state[0][self.drone_id]
        print(f'current location: {locations[drone.pos].type}: {drone.pos}')
        print(f'current drone status: {drone.status}')
        location_type = locations[self.landing_location].type
        print(f'target location: {locations[self.landing_location].type}')
        if location_type == 'P':
            inventory = state[1].get(self.landing_location)
            status = 'boxes left' if inventory else 'empty'
            print(f'inventory status: {status}')