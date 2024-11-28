# 梯度提升回归模型

import numpy as np
import matplotlib.pyplot as plt
from sklearn.ensemble import GradientBoostingRegressor
import joblib

# 给定的数据
rpm_values = np.array([5, 10, 15, 20, 25, 30, 35, 40, 45, 50])
high_values = np.array([6.4, 6.3, 6.3, 6.3, 6.3, 6.2, 6.2, 6.2, 6.2, 6.1])
sow_seed_values = np.array([7.86, 8.16, 7.90, 7.82, 7.77, 7.9, 8.35, 8.17, 8.03, 7.93, 7.75])

# 创建用于绘制拟合曲线的 RPM 范围
rpm_range = np.linspace(min(rpm_values), max(rpm_values), 500)

# 拟合梯度提升回归模型
gbr_model = GradientBoostingRegressor(n_estimators=500)
gbr_model.fit(rpm_values.reshape(-1, 1), high_values)
gbr_fit = gbr_model.predict(rpm_range.reshape(-1, 1))

# 保存模型到文件
joblib.dump(gbr_model, 'gradient_boosting_model.pkl')
print("Model saved successfully.")


# 绘制数据点
plt.scatter(rpm_values, high_values, color='black', label='Data Points')

# 绘制梯度提升回归拟合曲线
plt.plot(rpm_range, gbr_fit, label='Gradient Boosting Regression')

plt.xlabel('RPM')
plt.ylabel('Best Inoculation Space (mm²)')
plt.title('Gradient Boosting Regression Fit for RPM and Best Inoculation Space')
plt.legend()
plt.savefig(r'.\梯度提升回归模型.png')
plt.show()

