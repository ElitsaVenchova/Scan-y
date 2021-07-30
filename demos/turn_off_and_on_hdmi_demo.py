import time
import subprocess

while True:
	subprocess.call('vcgencmd display_power 1', shell=True)
	time.sleep(5)
	subprocess.call('vcgencmd display_power 0', shell=True)
	time.sleep(5)
