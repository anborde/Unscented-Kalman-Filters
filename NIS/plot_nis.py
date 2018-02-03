import pandas as pd
import matplotlib.pyplot as plt


# Plotting NIS for LIDAR measurments

data_laser = pd.read_csv('NIS_laser.csv')

# For LIDAR degree of freedom = 2 hence the Chi-Square value is 5.991
chi_value_laser = 5.991

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax1.plot(data_laser, label='LIDAR Data')
ax1.axhline(y=chi_value_laser, color='r', linestyle='-', label='Chi Square(0.05)')
ax1.title.set_text('NIS:LIDAR')
ax1.legend()

# Plotting NIS for LIDAR measurments

data_radar = pd.read_csv('NIS_radar.csv')

# For RADAR degree of freedom = 3 hence the Chi-Square value is 7.815
chi_value_radar = 7.815

ax2 = fig.add_subplot(212)
ax2.plot(data_radar, label='RADAR Data')
ax2.axhline(y=chi_value_radar, color='r', linestyle='-', label='Chi Square(0.05)')
ax2.title.set_text('NIS:RADAR')
ax2.legend()

plt.show()



