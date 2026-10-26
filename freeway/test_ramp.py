"""Ramp movement, storage, FIFO and replay-source checks."""
import json
from pathlib import Path
import unittest
from .ramp import RampRoad


class RampMotionTests(unittest.TestCase):
    def test_mainline_entry_restores_speed_and_following(self):
        from .model import FreewaySimulation
        from .behavior import Vehicle
        config=json.loads((Path(__file__).parent/'configs/default.json').read_text())
        sim=FreewaySimulation(config,'closure')
        sim.queues['ramp']=[(0,10)]
        sim.ramp.cars=[Vehicle(10,'ramp',0,0,sim.ramp.length-.6,.05,-1)]
        lead=Vehicle(11,'main',0,0,350,15,2)
        sim.vehicles=[lead]
        self.assertTrue(sim.try_enter('ramp',2,20))
        car=next(c for c in sim.vehicles if c.id==10)
        self.assertEqual(car.v,15)
        self.assertEqual(car.entry_speed,15)
        # Available headway allows the newly merged car to accelerate,
        # with the same IDM bound as the rest of the mainline.
        updates=sim.compute_updates(400,20)
        self.assertGreater(updates[10][1],15)
        self.assertGreater(updates[10][2],0)
        self.assertLessEqual(updates[10][2],1.2)
        self.assertEqual(sim.ramp.cars,[])

    def test_unsafe_mainline_gap_still_blocks_entry(self):
        from .model import FreewaySimulation
        from .behavior import Vehicle
        config=json.loads((Path(__file__).parent/'configs/default.json').read_text())
        sim=FreewaySimulation(config,'closure')
        sim.queues['ramp']=[(0,10)]
        sim.ramp.cars=[Vehicle(10,'ramp',0,0,sim.ramp.length-.6,.05,-1)]
        sim.vehicles=[Vehicle(11,'main',0,0,244,28,2)]
        self.assertFalse(sim.try_enter('ramp',2,20))
        self.assertEqual(len(sim.ramp.cars),1)
        self.assertEqual(sim.queues['ramp'],[(0,10)])

    def test_drive_stop_storage_and_clear(self):
        for dt in (.05,.025):
            road=RampRoad({'ramp_length_m':40,'ramp_desired_speed_mps':12})
            queue=[(0,i) for i in range(12)]
            moved=False
            for step in range(round(90/dt)):
                road.admit(queue,step*dt);road.advance(dt)
                moved=moved or any(c.v>2 for c in road.cars)
            self.assertTrue(moved)
            self.assertLess(len(road.cars),len(queue))
            self.assertTrue(road.ready(0))
            self.assertLess(road.cars[0].v,.01)
            self.assertGreaterEqual(road.minimum_gap,1.9)
            released=[]
            for step in range(round(180/dt)):
                road.admit(queue,90+step*dt)
                if queue and road.ready(queue[0][1]):
                    released.append(road.remove_front().id);queue.pop(0)
                road.advance(dt)
            self.assertEqual(released,list(range(12)))
            self.assertEqual(road.cars,[])
            self.assertAlmostEqual(road.distance,12*40,places=5)


def audit_ramp_frames(run,config):
    old={}; entered=set(); moving=0
    for frame in run['frames']:
        cars=frame['ramp_vehicles']
        assert frame['ramp_queue']==len(cars)+frame['ramp_upstream']
        road_ids={c[0] for c in frame['vehicles']}
        assert not road_ids.intersection(c[0] for c in cars)
        for a,b in zip(cars,cars[1:]):
            assert a[1]-b[1]>=5-1e-3
        for car in cars:
            vid,s,v,born=car
            assert 0<=s<=config.get('ramp_length_m',200) and v>=0
            assert vid not in entered
            assert vid not in old or s>=old[vid][0]
            old[vid]=(s,frame['time']);moving+=v>1
        for car in frame['vehicles']:
            if car[3]=='ramp':
                assert car[0] in old, 'Ramp entry without traversing ramp'
                entered.add(car[0])
    return {'moving_samples':moving,'distinct_mainline_entries':len(entered),'no_overlap_and_identity_continuity':'passed'}


if __name__=='__main__':
    unittest.main()
