from drone_delivery.objects import Drone, House, Pharmacy, Box


def clone_inventory(inventory):
    inventory_clone = {box_id: box for box_id, box in inventory.items()}
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
        inventories_clone = [clone_inventory(inventory) for inventory in state.inventories]
        return (drones_clone, inventories_clone)

    def apply(self, state):
        pass


class Pickup(Action):
    def __init__(self, drone_id, box_id):
        super().__init__(name='pickup')
        self.drone_id = drone_id
        self.box_id = box_id

    def applicable(self, state):
        # return true if drone has no object AND currently at a pharmacy with inventory
        return not state.drones[self.drone_id].has_object() and state.inventories[self.location_id] != []

    def apply(self, state):
        super(Pickup, self).clone(state)
        # assign object to drone and remove from inventory
        self.resulting_state
        # modify self.resulting_state
        return self.resulting_state

class Dropoff(Action):
    def __init__(self, drone_id, box_id):
        super().__init__(name='dropoff')
        self.drone_id = drone_id
        self.box_id = box_id

    def applicable(self, state):
        # return true if drone has object AND is at object's dropoff location
        pass

    def apply(self, state):
        super(Dropoff, self).clone(state)
        # modify self.resulting_state
        # reset drone status to None
        return self.resulting_state

class Move(Action):
    def __init__(self, drone_id, takeoff_location, landing_location):
        super().__init__(name='move')
        self.drone_id = drone_id
        self.takeoff_location = takeoff_location
        self.landing_location = landing_location

    def applicable(self, state):
        # drone is carrying object AND not at object's end pos AND end pos is the object's end pos
        # drone has no object AND a pharmacy still has inventory
        # drone start pos does not equal end pos
        # start pos location and end location are not both houses
        # return true if all of the above are true
        pass

    def apply(self, state):
        super(Move, self).clone(state)
        # modify self.resulting_state
        # set drone pos to end pos
        return self.resulting_state
