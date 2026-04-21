#0x0201 topic:/robot/robotstatus
uint8 robot_id#本机器人 ID
uint8 robot_level#机器人等级
uint16 current_hp#机器人当前血量
uint16 maximum_hp#机器人血量上限
uint16 shooter_barrel_cooling_value#机器人射击热量每秒冷却值
uint16 shooter_barrel_heat_limit#机器人射击热量上限
uint16 chassis_power_limit#机器人底盘功率上限
#电源管理模块的输出情况：
uint8 power_management_gimbal_output#gimbal 口输出：0 为无输出，1 为 24V 输出
uint8 power_management_chassis_output#chassis 口输出：0 为无输出，1 为 24V 输出
uint8 power_management_shooter_output#shooter 口输出：0 为无输出，1 为 24V 输出