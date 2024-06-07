#draw a picture of a heart
import matplotlib.pyplot as plt
import numpy as np
x = np.linspace(-2, 2, 1000)
y = np.sqrt(1-(abs(x)-1)**2)
plt.plot(x, y, color = 'red')
plt.plot(x, -y, color = 'red')
plt.title('Heart')
plt.show()
#end of test.py



