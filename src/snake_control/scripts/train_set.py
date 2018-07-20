#This file is used to obtain training angles between the snake and tager location

# Up is positive X direction
# Left is positive Y direction

import random
import math

rangeL = 0.0
rangeU = 20.0
f = open('target_data.txt', 'w')
tar_location = (12.50,10.00)   # Target set above centre to train more on positive rel_x data
for _ in range(500):
	dist = 0.0
	while (dist<1.5):
		i = random.uniform(rangeL,rangeU)
		j = random.uniform(rangeL,rangeU)
		rel_x = tar_location[0] - i
		rel_y = tar_location[1] - j
		dist = math.sqrt((rel_x)*(rel_x) + (rel_y)*(rel_y))
	alphaR = alphaL = 0
	if rel_x < 0:
		if rel_y > 0:
			alphaL = 90
			alphaR = -180
		else:
			alphaL = 180
			alphaR = -90
	else:
		val = rel_y / dist
		alpha = math.asin(val)*(180/math.pi)
		if alpha > 0:
			alphaL = alpha
			alphaR = -180
		else:
			alphaL = 180
			alphaR = alpha

	print(rel_x,rel_y,alphaL,alphaR)
	f.write(str(rel_x) + ',' + str(rel_y) + ',' + str(alphaL) + ',' + str(alphaR) + '\n')

f.close()

