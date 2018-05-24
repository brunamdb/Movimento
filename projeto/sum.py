import numpy as np


a = [[[  0   ,0]],[[  0 ,479]],[[805 ,479]],[[855   ,0]]]



b = np.sum(a, axis = 1)

print(b)

print(np.argmax(b))
