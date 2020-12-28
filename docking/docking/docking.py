import asyncio

from drone import Drone
from target import Target

async def dock():
    target=Target (0,0,0,0)
    drone = Drone(3,target)
    await drone.connect_gazebo()
    await drone.takeoff()
    await drone.initiate_docking(1)
    
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(dock())