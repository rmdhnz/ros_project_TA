#!/usr/bin/env python3
import pandas as pd
import numpy as np
from statsmodels.tsa.stattools import adfuller
from statsmodels.tsa.arima.model import ARIMA
import matplotlib.pyplot as plt
import control as ctrl
from statsmodels.graphics.tsaplots import plot_acf,plot_pacf
# Memuat data IMU (misal dalam bentuk dataframe)
data = pd.read_csv('../../../odom_data.csv', header=None)
odom_data = data[0]  # Ambil salah satu kolom misalnya percepatan linear x
result = adfuller(odom_data)
for key, value in result[4].items():
    print(f'Critical Value {key}: {value}')

differenced_data = odom_data.diff().dropna()
differenced_data = differenced_data.diff().dropna()
result = adfuller(differenced_data)

print('ADF Statistic:', result[0])
print('p-value:', result[1])
for key, value in result[4].items():
    print(f'Critical Value {key}: {value}')

# Membuat model ARIMA
# best_model = None
# mse_val = None
# for i in range(5) : 
#     for j in range(5) :
#         model = ARIMA(odom_data, order=(i+1, 2, j+1))  # order(p, d, q) ARIMA
#         model_fit = model.fit()
#         predictions = model_fit.predict(start=0, end=len(odom_data)-1, typ='levels')
#         if not mse_val : 
#             mse_val = np.mean(np.square(predictions - odom_data))
#             best_model = model_fit
#         else : 
#             mse_curr = np.mean(np.square(predictions-odom_data))
#             if mse_curr < mse_val :
#                 mse_val = mse_curr
#                 best_model = model_fit


model = ARIMA(odom_data,order=(4,2,2))
model_fit = model.fit()


# Melihat hasil summary
print(model_fit.summary())
print(f"Koefisien AR : {model_fit.arparams}")
print(f"Koefisien AR : {model_fit.maparams}")
tf = ctrl.TransferFunction(model_fit.maparams,model_fit.arparams)
print(tf)
sys_ss = ctrl.tf2ss(tf)
print("A matrix:\n", sys_ss.A)
print("B matrix:\n", sys_ss.B)
print("C matrix:\n", sys_ss.C)
print("D matrix:\n", sys_ss.D)
# Prediksi data
predictions = model_fit.predict(start=0, end=len(odom_data)-1, typ='levels')
print("MSE value : {:.5f}".format(np.mean(np.square(predictions - odom_data))))
# Plot prediksi
plt.plot(predictions,label="ARIMA")
plt.plot(odom_data,label="Sensor")
plt.grid(True)
plt.legend(loc="upper left")
plt.show()
