import carla

carla_host = '172.17.0.5'
carla_port = 2000

client = carla.Client(carla_host, carla_port)
client.set_timeout(4.0)
sim_world = client.get_world()

settings = sim_world.get_settings()

while True:
    sim_world.wait_for_tick()
    print("server ticked!")
