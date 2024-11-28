import numpy as np
import joblib

# 加载保存的模型
gbr_model = joblib.load('gradient_boosting_model.pkl')

# 输入一个新的 RPM 值
new_rpm_value = 25  # 新的 RPM 数据

# 对新的 RPM 值进行预测，得到最佳挡板高度
predicted_high_value = gbr_model.predict(np.array([[new_rpm_value]]))
predicted_high_value = round(float(predicted_high_value[0]), 1)

# 打印预测结果
print(predicted_high_value)

