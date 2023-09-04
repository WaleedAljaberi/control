angle = 0
new_yaw = 320
rotating_margin = 45
if angle - rotating_margin < 0 and new_yaw > 360 - rotating_margin:
    new_yaw = new_yaw - 360

print(new_yaw)