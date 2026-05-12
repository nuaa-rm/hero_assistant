//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_MESSAGE_H
#define ROBOT_SERIAL_ROBOT_MESSAGE_H

#include "msg_serialize.h"

/* 用message_data定义全部要发送给下位机的消息的结构体 */
/* 0x05开头是发送给下位机的数据 */
message_data Velocity{ //0x0501
    float v_x, v_y, v_w;
};
message_data Action{ //0x0502
    float max_pitch_w,min_pitch_w;
    uint8_t mode;
    uint8_t spin;
    uint8_t patrol;
    uint8_t robot_aim;
    uint8_t chasing;
};
message_data Whitelist{ //0x0506
    uint8_t whitelist[12];
};
message_data DistanceInfo{ //0x0606
    float distance;
};

/* 接收和发送的裁判系统指令按裁判系统串口手册中的命令字来命名 */
message_data game_status_t{ //0x0001
    uint8_t game_progress;
    uint16_t stage_remain_time;
};
message_data game_robot_HP_t{ //0x0003
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
};
message_data event_data_t{ //0x0101
    uint32_t inside_recovery : 1;
    uint32_t outside_recovery : 1;
    uint32_t recovery_rmul : 1;
    uint32_t small_energy_organ_status : 1;
    uint32_t big_energy_organ_status : 1;
    uint32_t central_plateau : 2;
    uint32_t trapezoidal_heights : 2;
    uint32_t time_of_dart_hit : 9;
    uint32_t target_of_dart_hit : 3;
    uint32_t center_rmul : 2;
    uint32_t our_fort : 2;
    uint32_t reserved : 7;
};
message_data dart_info_t{ //0x0105
    uint16_t dart_hit_target : 3;
    uint16_t dart_count_aim : 3;
    uint16_t dart_selected_target : 2;
    uint16_t reserved : 8;
};
message_data robot_pos_t{ //0x0203
    float x;
    float y;
    float angle;
};
message_data buff_t{ //0x0204
    uint8_t defence_buff;
    uint8_t remaining_energy;
};
message_data hurt_data_t{ //0x0206
 uint8_t armor_id : 4;
 uint8_t hp_deduction_reason : 4;
}; 

message_data projectile_allowance_t{ //0x0208
    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;
};
message_data rfid_status_t{ //0x0209
    uint32_t base : 1;
    uint32_t our_central_plateau : 1;
    uint32_t his_central_plateau : 1;
    uint32_t our_trapezoidal_heights : 1;
    uint32_t his_trapezoidal_heights : 1;
    uint32_t our_cross_feipo_front : 1;
    uint32_t our_cross_feipo_back : 1;
    uint32_t his_cross_feipo_front : 1;
    uint32_t his_cross_feipo_back : 1;
    uint32_t our_cross_central_below : 1;
    uint32_t our_cross_central_up : 1;
    uint32_t his_cross_central_below : 1;
    uint32_t his_cross_central_up : 1;
    uint32_t our_cross_road_below : 1;
    uint32_t our_cross_road_up : 1;
    uint32_t his_cross_road_below : 1;
    uint32_t his_cross_road_up : 1;
    uint32_t our_fort : 1;
    uint32_t outpost : 1;
    uint32_t inside_recovery : 1;
    uint32_t outside_recovery : 1;
    uint32_t our_big_resources : 1;
    uint32_t his_big_resources : 1;
    uint32_t center : 1;
    uint32_t his_fort : 1;
    uint32_t reserved : 7;
};
message_data ground_robot_position_t{ //0x020B
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
};
message_data sentry_info_t{ //0x020D
    uint32_t amount_of_get_ammunition : 11;
    uint32_t number_of_get_ammunition : 4;
    uint32_t number_of_get_blood : 4;
    uint32_t confirm_free_resurrection : 1;
    uint32_t confirm_resurrection_immediately : 1;
    uint32_t cost_of_resurrection : 10;
    uint32_t reserved1 : 1;
    uint16_t if_out_fight : 1;
    uint16_t projectile_allowance_17mm_of_team : 11;
    uint16_t reserved2 : 4;
};
message_data robot_interaction_data_t{ //0x0301
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[61];
};
message_data sentry_cmd_t{ //0x0301子内容 哨兵自主决策指令：0x0120
    uint32_t sentry_cmd;
};
message_data sentry_decision_data_t{ //0x0301子内容 哨兵自主决策指令：0x0120
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint32_t sentry_cmd;
};
message_data lidarstation_data_t{ //接收雷达站从裁判系统发来的消息
    uint8_t attack_enhance;
    uint16_t robot_x[5];
    uint16_t robot_y[5];
    uint8_t robot_z[5];
};
message_data map_command_t{ //0x0303
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
};
message_data map_data_t{ //0x0307
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
};
message_data custom_info_t{ //0x0308
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
};

message_data AngleError{ //云台和底盘角度差 0x0601
    float angle_error;
    float gimbal_error;
};

/* 打包接收来自裁判系统的数据 */
message_data data_received_packag_200hz_t{ //0x0606
    game_robot_HP_t robot_HP; //0x0003
    hurt_data_t hurt_data;
    projectile_allowance_t projectile_allowance; //0x0208
    rfid_status_t rfid_status; //0x0209
    AngleError angle_error; //0x0601
};

message_data data_received_packag_1hz_t{ //0x0607
    game_status_t game_status; //0x0001
    event_data_t event_data; //0x0101
    dart_info_t dart_info; //0x0105
    robot_pos_t robot_pos; //0x0203
    buff_t buff; //0x0204
    ground_robot_position_t ground_robot_position; //0x020B
    sentry_info_t sentry_info; //0x020D
    lidarstation_data_t lidarstation_data; //0x0301
    map_command_t map_command; //0x0303

};
#endif //ROBOT_SERIAL_ROBOT_MESSAGE_H
