import asyncio

from drone import Drone

async def dock():
    drone = Drone()
    await drone.connect_gazebo()
    await drone.takeoff()
    await drone.initiate_docking()
    
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(dock())