import agv_with_stm32
import time

def main():
	agv = agv_with_stm32.AGV()
	agv.set_velocities(-1000,0,0)
	agv.send_velocities()
	#agv.send_velocities()
	time.sleep(2)
	#agv.set_velocities(200,0,0)
	#agv.send_velocities()
	#time.sleep(5)
	agv.set_velocities(0,0,0)
	agv.send_velocities()
	agv.close_com()

if __name__ == '__main__':main()
