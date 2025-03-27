# 耗时分析
import matplotlib.pyplot as plt

labels = ['<10ms', '10~20ms', '20~30ms', '30~40ms', '>40ms']
sizes = [112274, 25883, 5663, 1163,264 ]

plt.figure(figsize=(8, 8))
plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=140)
plt.title('time')
plt.axis('equal')
plt.show()
