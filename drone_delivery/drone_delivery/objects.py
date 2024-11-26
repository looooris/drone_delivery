class Drone():
    def __init__(self, drone_id, pos, status=None, plan=[]):
        self.drone_id = drone_id
        self.pos = pos
        self.status = status
        self.plan = plan

    def has_object(self):
        return self.status != None
    
    def clone(self):
        return Drone(drone_id=self.drone_id, pos=self.pos[:], status=self.status, plan=self.plan)

    
class Box():
    def __init__(self, box_id, pickup_pos, dropoff_pos, priority=None):
        self.box_id = box_id
        self.pickup_pos = pickup_pos
        self.dropoff_pos = dropoff_pos
        self.priority = priority

    def returnPickup(self):
        return self.pickup_pos

class Location(object):
    def __init__(self, location_id, pos):
        self.location_id = location_id
        self.pos = pos

class Pharmacy(Location):
    # has inventory list of boxes. completing order will remove box from inventory
    def __init__(self,location_id, pos):
        super().__init__(location_id, pos)
        self.type = 'P'

    """ not sure about using these
    def place_order(self, box):
        self.inventory.append(box)

    def complete_order(self, box):
        self.inventory.remove(box)

    def all_orders_completed(self):
        return self.inventory == []
    """
    

class House(Location):
    # doesn't have any inventory. object can only be dro
    def __init__(self, pos):
        self.pos = pos
        self.type = 'H'