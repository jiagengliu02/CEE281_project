"""Single-lane IDM ramp, upstream storage, and a downstream metering stop.

Coordinates measure front-bumper distance along the ramp. The downstream
handoff is within one metre of the merge point; safe mainline insertion is
checked independently. No lateral steering dynamics are implied.
"""
from .behavior import Vehicle, idm_acceleration, motion


class RampRoad:
    def __init__(self, config):
        self.length = config.get('ramp_length_m', 200.0)
        self.speed = config.get('ramp_desired_speed_mps', 12.0)
        self.cars = []
        self.distance = 0.0
        self.minimum_gap = float('inf')

    def admit(self, queue, t):
        active = {c.id for c in self.cars}
        pending = next(((born, vid) for born, vid in queue if vid not in active), None)
        if pending is None:
            return
        rear = self.cars[-1] if self.cars else None
        if rear and rear.x - rear.length < 2:
            return
        born, vid = pending
        # Start at rest to preserve a safe entrance even when the ramp is full.
        self.cars.append(Vehicle(vid, 'ramp', born, t, 0, 0, -1, desired_speed=self.speed))

    def ready(self, vid):
        return bool(self.cars and self.cars[0].id == vid and self.cars[0].x >= self.length - 1)

    def remove_front(self):
        car = self.cars.pop(0)
        self.distance += max(0, self.length - car.x)
        return car

    def advance(self, dt):
        updates = []
        for i, car in enumerate(self.cars):
            lead = self.cars[i-1] if i else None
            # A virtual stop 1.5 m past the endpoint leaves 0.5 m of
            # clearance at IDM's 2 m standstill gap.
            gap = lead.x - lead.length - car.x if lead else self.length + 1.5 - car.x
            lead_v = lead.v if lead else 0.0
            a = idm_acceleration(car.v, gap, car.v-lead_v, self.speed)
            a = max(-6.0, min(1.2, a))
            x, v = motion(car.x, car.v, a, dt)
            ceiling = updates[i-1][0] - lead.length if lead else self.length
            assert x <= ceiling + 1e-8, (car.id,car.x,car.v,x,ceiling,i)
            assert x >= car.x and v >= 0
            if lead:
                self.minimum_gap = min(self.minimum_gap, ceiling-x)
            self.distance += x-car.x
            updates.append((x,v,a))
        for car,(x,v,a) in zip(self.cars,updates):
            car.x,car.v,car.a=x,v,a

    def snapshot(self):
        return [[c.id,round(c.x,4),round(c.v,4),c.entered] for c in self.cars]
