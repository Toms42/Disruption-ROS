import time
import random

while True:
	with open('communicate.dat', 'w+') as f:
		list1 = ["Snake", "Frog", "Bird"]
		rand = random.choice(list1)
		f.write(rand + ":True:\n")
		print(rand + ":True:")
		#SPEND AS LITTLE TIME AS POSSIBLE DURING WRITE OR ELSE GARTHS CODE WONT READ
	time.sleep(1)

