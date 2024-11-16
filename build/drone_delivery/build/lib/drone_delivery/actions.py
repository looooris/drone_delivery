from drone_delivery.objects import Drone, House, Pharmacy, Box


def clone_inventory(inventory):
    inventory_clone = (box for box in inventory)
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
        drones_clone = [Drone.clone(drone) for drone in state.drones]
        inventories_clone = {clone_inventory(inventory) for inventory in state.inventories}
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
        inventory = state.inventories[self.pharmacy_id]

        return not state.drones[self.drone_id].has_object() and len(inventory) > 0

    def apply(self, state):
        resulting_state = super(Pickup, self).clone(state)
        
        # remove box from inventory
        resulting_state.inventories[self.pharmacy_id].discard(self.box)

        # assign box to drone
        resulting_state.drones[self.drone_id].status = self.box

        return resulting_state


class Dropoff(Action):
    def __init__(self, drone_id, box):
        super().__init__(name='dropoff', drone_id=drone_id)
        self.box = box

    def applicable(self, state, locations=None):
        # return true if drone has box AND drone is at box's dropoff location
        drone = state.drones[self.drone_id]
        return drone.has_object() and drone.pos == self.box.dropoff

    def apply(self, state):
        resulting_state = super(Dropoff, self).clone(state)

        # reset drone status to None
        resulting_state.drones[self.drone_id].status = None

        return resulting_state


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
        drone = state.drones[self.drone_id]
        box = drone.status

        # if drone carrying box and box dropoff pos == landing position
        if drone.has_object() and box.dropoff_pos == self.landing_location:
            return True
        
        # drone not carrying box AND landing pharmacy still has inventory
        if not drone.has_object() and locations[self.landing_location].type == 'P':
            # get the pharmacy id of the box's dropoff location
            inventory = state.inventories[self.landing_location]
            if len(inventory) > 0:
                return True
       
        return False

    def apply(self, state):
        resulting_state = super(Move, self).clone(state)

         # set drone pos to end pos
        resulting_state.drones[self.drone_id].pos = self.landing_location

        return resulting_state
