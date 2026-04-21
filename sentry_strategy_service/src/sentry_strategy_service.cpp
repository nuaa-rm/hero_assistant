/**
 * @file sentry_strategy_service.cpp
 * @brief 哨兵策略服务实现文件
 * @details 该文件实现了哨兵机器人的目标选择策略服务，包括自瞄信息处理、全向感知融合和目标优先级判断
 * @author elsa
 * @date 2025-05-14
 * @version 1.0
 */


#include "sentry_strategy_service/sentry_strategy_service.hpp"

/**
 * @brief 哨兵策略服务构造函数
 * @details 初始化ROS2节点，创建订阅者和服务，设置消息存储容器
 */
SentryStrategyService::SentryStrategyService() : Node("sentry_strategy_service")
{
    RCLCPP_INFO(this->get_logger(), "Sentry Strategy Service start");

    // 发布敌人位置信息
    enemy_chasing_pub = this->create_publisher<rm_interfaces::msg::EnemyChasing>("/robot/enemy_chasing", 10);
    // 订阅自瞄策略信息
    autoaim_strategy_sub = this->create_subscription<rm_interfaces::msg::AutoaimStrategy>(
        "/robot/strategy",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&SentryStrategyService::AutoaimStrategyCallback, this, std::placeholders::_1));

    // 订阅全向感知信息
    occlusion_detector_sub = this->create_subscription<rm_interfaces::msg::EnemyPositions>(
        "/multi_combine/enemies",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&SentryStrategyService::OcclusionCallback, this, std::placeholders::_1));

    // 创建哨兵策略服务
    sentry_strategy_service = this->create_service<rm_interfaces::srv::EnemyStrategist>(
        "/robot_strategist/choose_robot", std::bind(&SentryStrategyService::SentryStrategyServiceCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));

    // 初始化敌人信息存储容器
    enemies = std::make_shared<rm_interfaces::msg::EnemyPositions>();
    enemies->enemies.reserve(7);
    occlusion_detector = std::make_shared<rm_interfaces::msg::EnemyPositions>();
    occlusion_detector->enemies.reserve(5);
    // std::cout << "enemies->enemies.size: " << enemies->enemies.size() << std::endl;
    // std::cout << "occlusion_detector->enemies.size: " << occlusion_detector->enemies.size() << std::endl;
    received_autoaim_strategy = std::make_shared<rm_interfaces::msg::AutoaimStrategy>();

    dog_cnt = 0;
    dog_cnt_threshold = 5.0 / 0.02;
    lost_delay_cnt = 0;
    if_in_lost_waiting = false;
    omni_detected = false;

    this->declare_parameter<std::vector<std::string>>("aim_priority", std::vector<std::string>{"1", "3", "4", "2", "sentry"});
    this->declare_parameter("if_default_chasing", true);

    this->get_parameter<std::vector<std::string>>("aim_priority", aim_priority);
    this->get_parameter("if_default_chasing", if_default_chasing);
    RCLCPP_INFO(this->get_logger(), "aim_priority: %s, %s, %s, %s, %s", aim_priority[0].c_str(),
        aim_priority[1].c_str(), aim_priority[2].c_str(), aim_priority[3].c_str(), aim_priority[4].c_str());
    RCLCPP_INFO(this->get_logger(), "default_chasing: %d", if_default_chasing);

    RCLCPP_INFO(this->get_logger(), "Sentry Strategy Service 初始化完成");
}

/**
 * @brief 将整数ID转换为字符串ID
 * @param int_id 整数ID (0-6)
 * @return 对应的字符串ID
 * @details 0->"1", 1->"2", 2->"3", 3->"4", 4->"sentry", 5->"outpost", 6->"base"
 */
std::string SentryStrategyService::int_id2string_id(int int_id)
{
    std::string string_id;
    switch (int_id)
    {
    case 0: string_id = "1";
        break;
    case 1: string_id = "2";
        break;
    case 2: string_id = "3";
        break;
    case 3: string_id = "4";
        break;
    case 4: string_id = "sentry";
        break;
    case 5: string_id = "outpost";
        break;
    case 6: string_id = "base";
        break;
    default: string_id = "0";
        break;
    }
    return string_id;
}

/**
 * @brief 将字符串ID转换为整数ID
 * @param string_id 字符串ID
 * @return 对应的整数ID，无效时返回-1
 * @details "1"->0, "2"->1, "3"->2, "4"->3, "sentry"->4, "outpost"->5, "base"->6
 */
int SentryStrategyService::string_id2int_id(std::string string_id)
{
    int int_id = -1;
    if (string_id == "1")
        int_id = 0;
    else if (string_id == "2")
        int_id = 1;
    else if (string_id == "3")
        int_id = 2;
    else if (string_id == "4")
        int_id = 3;
    else if (string_id == "sentry")
        int_id = 4;
    else if (string_id == "outpost")
        int_id = 5;
    else if (string_id == "base")
        int_id = 6;

    return int_id;
}

/**
 * @brief 哨兵策略服务回调函数
 * @param request 服务请求，包含所有敌人信息
 * @param response 服务响应，返回选择的目标信息
 * @details 实现哨兵机器人的目标选择策略：
 * 1. 验证输入数据有效性
 * 2. 融合自瞄和全向感知信息
 * 3. 根据血量优先级选择目标
 * 4. 返回选择的目标信息
 */
void SentryStrategyService::SentryStrategyServiceCallback(
    const std::shared_ptr<rm_interfaces::srv::EnemyStrategist::Request> request,
    std::shared_ptr<rm_interfaces::srv::EnemyStrategist::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "receive autoaim request");
    dog_cnt = 0;

    /* 获取服务消息并存储 */
    enemy_size = request->all_enemy.enemies.size();
    if (enemy_size < 7)
    {
        response->err_code = 0;
        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = false;
        enemy_chasing_pub->publish(enemy_chasing_msg);
        RCLCPP_INFO(this->get_logger(), "自瞄信息数量有误，默认听自瞄的");
        return;
    }
    
    /* 信息存储：根据string_id存入对应的enemies位置中 */
    for (int i = 0; i < 7; i++)
    {
        //根据当前string_id转为int格式，按顺序存入enemies中
        int enemy_id_index = string_id2int_id(request->all_enemy.enemies[i].id);
        if (enemy_id_index == -1)
        {
            response->err_code = 0;
            rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
            enemy_chasing_msg.is_in_chase = false;
            enemy_chasing_pub->publish(enemy_chasing_msg);
            RCLCPP_INFO(this->get_logger(), "获取的string_id信息有误，默认听自瞄的");
            return;
        }
        if (request->all_enemy.enemies[i].is_detected) {
            RCLCPP_INFO(this->get_logger(), "自瞄识别id: %s", request->all_enemy.enemies[i].id.c_str());
        }
        
        // 处理前哨站和基地信息
        if(enemy_id_index == 5 || enemy_id_index == 6) //outpost/base
        {
            enemies->enemies[enemy_id_index].is_detected = request->all_enemy.enemies[i].is_detected;
            enemies->enemies[enemy_id_index].id = request->all_enemy.enemies[i].id;
        }
        else //robot
        {
            enemies->enemies[enemy_id_index].is_detected = request->all_enemy.enemies[i].is_detected;
            
            // 融合全向感知信息：当自瞄未检测到但全向感知检测到时，使用全向感知信息
            if(!enemies->enemies[enemy_id_index].is_detected && occlusion_detector->enemies[enemy_id_index].is_detected)
            {
                //只有全向感知识别到
                enemies->enemies[enemy_id_index].is_detected = occlusion_detector->enemies[enemy_id_index].is_detected;
                enemies->enemies[enemy_id_index].occlusion = true;
                enemies->enemies[enemy_id_index].main = false;
                enemies->enemies[enemy_id_index].id = occlusion_detector->enemies[enemy_id_index].id;
                enemies->enemies[enemy_id_index].attack_enhance = occlusion_detector->enemies[enemy_id_index].attack_enhance;
                enemies->enemies[enemy_id_index].pose.position.x = occlusion_detector->enemies[enemy_id_index].pose.position.x;
                enemies->enemies[enemy_id_index].pose.position.y = occlusion_detector->enemies[enemy_id_index].pose.position.y;
                enemies->enemies[enemy_id_index].pose.position.z = occlusion_detector->enemies[enemy_id_index].pose.position.z;
                enemies->enemies[enemy_id_index].pose.orientation.x = occlusion_detector->enemies[enemy_id_index].pose.orientation.x;
                enemies->enemies[enemy_id_index].pose.orientation.y = occlusion_detector->enemies[enemy_id_index].pose.orientation.y;
                enemies->enemies[enemy_id_index].pose.orientation.z = occlusion_detector->enemies[enemy_id_index].pose.orientation.z;
                enemies->enemies[enemy_id_index].pose.orientation.w = occlusion_detector->enemies[enemy_id_index].pose.orientation.w;
                RCLCPP_INFO(this->get_logger(), "全向感知识别id: %s", enemies->enemies[enemy_id_index].id.c_str());
            }
            else
            {
                // 使用自瞄信息
                enemies->enemies[enemy_id_index].occlusion = false;
                enemies->enemies[enemy_id_index].main = request->all_enemy.enemies[i].main;
                enemies->enemies[enemy_id_index].id = request->all_enemy.enemies[i].id;
                enemies->enemies[enemy_id_index].attack_enhance = request->all_enemy.enemies[i].attack_enhance;
                enemies->enemies[enemy_id_index].pose.position.x = request->all_enemy.enemies[i].pose.position.x;
                enemies->enemies[enemy_id_index].pose.position.y = request->all_enemy.enemies[i].pose.position.y;
                enemies->enemies[enemy_id_index].pose.position.z = request->all_enemy.enemies[i].pose.position.z;
                enemies->enemies[enemy_id_index].pose.orientation.x = request->all_enemy.enemies[i].pose.orientation.x;
                enemies->enemies[enemy_id_index].pose.orientation.y = request->all_enemy.enemies[i].pose.orientation.y;
                enemies->enemies[enemy_id_index].pose.orientation.z = request->all_enemy.enemies[i].pose.orientation.z;
                enemies->enemies[enemy_id_index].pose.orientation.w = request->all_enemy.enemies[i].pose.orientation.w;
            }
            enemies->enemies[enemy_id_index].hp = received_autoaim_strategy->enemy_blood[enemy_id_index];
            enemies->enemies[enemy_id_index].invincible = received_autoaim_strategy->enemy_invincible[enemy_id_index];
        }
    }

    /* 全向感知丢失目标延时保护，丢失目标0.5s内会持续让主相机追踪先前的目标 */
    if(omni_detected && !enemies->enemies[omni_detected_id_index].is_detected)
    {
        if_in_lost_waiting = true;

        response->err_code = 1;
        response->position.is_detected = enemies->enemies[omni_detected_id_index].is_detected; //?
        response->position.occlusion = false;
        response->position.invincible = received_autoaim_strategy->enemy_invincible[omni_detected_id_index];
        response->position.main = false; //?
        response->position.id = enemies->enemies[omni_detected_id_index].id;
        response->position.hp = received_autoaim_strategy->enemy_blood[omni_detected_id_index];
        response->position.pose.position.x = 0.0;
        response->position.pose.position.y = 0.0;
        response->position.pose.position.z = 0.0;
        response->position.pose.orientation.x = 0.0;
        response->position.pose.orientation.y = 0.0;
        response->position.pose.orientation.z = 0.0;
        response->position.pose.orientation.w = 1.0;
        response->position.attack_enhance = enemies->enemies[omni_detected_id_index].attack_enhance;

        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = false;
        enemy_chasing_pub->publish(enemy_chasing_msg);
        RCLCPP_INFO(this->get_logger(), "丢失等待中，强制追踪id: %s", response->position.id.c_str());
    }

    /* 选车逻辑 */
    //先根据血量多少选目标
    double min_blood = 500.0;
    int min_idx = -1;
    for (int i = 0; i < 7; i++) //是否需要打以及按血量排序
    {
        is_lock[i] = enemies->enemies[i].is_detected * received_autoaim_strategy->whitelist[i];
         //* received_autoaim_strategy->enemy_invincible[i]; // DEBUG 上场前开下来
        if (is_lock[i] && i != 5 && i != 6) //需要打，且是车，前哨站和基地不参与血量优先的选择判断
        {
            enemy_blood_to_select[i] = received_autoaim_strategy->enemy_blood[i]; //存敌方当前血量
        }
        else
            enemy_blood_to_select[i] = 1500.0;
        if (enemy_blood_to_select[i] < min_blood)
        {
            min_blood = enemy_blood_to_select[i];
            min_idx = i;
        }
    }
    if (min_idx == -1)
    {
        response->err_code = 2; //自瞄关闭
        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = false;
        enemy_chasing_pub->publish(enemy_chasing_msg);
        RCLCPP_WARN(this->get_logger(), "当前自瞄已关闭");
        return;
    }
    
    // 策略1：追击血量少于100的敌人
    if (min_blood <= 100) //追击血量最少且小于100的
    {
        response->err_code = 1;
        response->id = int_id2string_id(min_idx);
        if (response->id == "0")
        {
            response->err_code = 0;
            rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
            enemy_chasing_msg.is_in_chase = false;
            enemy_chasing_pub->publish(enemy_chasing_msg);
            RCLCPP_INFO(this->get_logger(), "获取的int_id信息有误，默认听自瞄的");
            return;
        }
        response->position.is_detected = enemies->enemies[min_idx].is_detected;
        response->position.occlusion = enemies->enemies[min_idx].occlusion;
        response->position.invincible = received_autoaim_strategy->enemy_invincible[min_idx];
        response->position.main = enemies->enemies[min_idx].main;
        response->position.id = enemies->enemies[min_idx].id;
        response->position.hp = received_autoaim_strategy->enemy_blood[min_idx];
        response->position.pose.position.x = enemies->enemies[min_idx].pose.position.x;
        response->position.pose.position.y = enemies->enemies[min_idx].pose.position.y;
        response->position.pose.position.z = enemies->enemies[min_idx].pose.position.z;
        response->position.pose.orientation.x = enemies->enemies[min_idx].pose.orientation.x;
        response->position.pose.orientation.y = enemies->enemies[min_idx].pose.orientation.y;
        response->position.pose.orientation.z = enemies->enemies[min_idx].pose.orientation.z;
        response->position.pose.orientation.w = enemies->enemies[min_idx].pose.orientation.w;
        response->position.attack_enhance = enemies->enemies[min_idx].attack_enhance;

        if(response->position.occlusion)
        {
            omni_detected = true;
            omni_detected_id_index = min_idx;
        }

        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = true;
        enemy_chasing_msg.aim_goal.pose.position.x = enemies->enemies[min_idx].pose.position.x;
        enemy_chasing_msg.aim_goal.pose.position.y = enemies->enemies[min_idx].pose.position.y;
        enemy_chasing_msg.aim_goal.pose.position.z = enemies->enemies[min_idx].pose.position.z;
        enemy_chasing_pub->publish(enemy_chasing_msg);

        RCLCPP_INFO(this->get_logger(), "追击血量小于100的敌人, id: %s, hp: %lu",
            enemies->enemies[min_idx].id.c_str(), enemies->enemies[min_idx].hp);
        RCLCPP_INFO(this->get_logger(), "position x y z: %f, %f, %f",
            response->position.pose.position.x, response->position.pose.position.y, response->position.pose.position.z);
    }
    // 策略2：追击英雄
    else if (is_lock[0]) //没有血量适合哨兵一下带走的，尝试追击英雄
    {
        response->err_code = 1;
        response->id = enemies->enemies[0].id;
        response->position.is_detected = enemies->enemies[0].is_detected;
        response->position.occlusion = enemies->enemies[0].occlusion;
        response->position.invincible = received_autoaim_strategy->enemy_invincible[0];
        response->position.main = enemies->enemies[0].main;
        response->position.id = enemies->enemies[0].id;
        response->position.hp = received_autoaim_strategy->enemy_blood[0];
        response->position.pose.position.x = enemies->enemies[0].pose.position.x;
        response->position.pose.position.y = enemies->enemies[0].pose.position.y;
        response->position.pose.position.z = enemies->enemies[0].pose.position.z;
        response->position.pose.orientation.x = enemies->enemies[0].pose.orientation.x;
        response->position.pose.orientation.y = enemies->enemies[0].pose.orientation.y;
        response->position.pose.orientation.z = enemies->enemies[0].pose.orientation.z;
        response->position.pose.orientation.w = enemies->enemies[0].pose.orientation.w;
        response->position.attack_enhance = enemies->enemies[0].attack_enhance;

        if(response->position.occlusion)
        {
            omni_detected = true;
            omni_detected_id_index = 0;
        }

        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = true;
        enemy_chasing_msg.aim_goal.pose.position.x = enemies->enemies[min_idx].pose.position.x;
        enemy_chasing_msg.aim_goal.pose.position.y = enemies->enemies[min_idx].pose.position.y;
        enemy_chasing_msg.aim_goal.pose.position.z = enemies->enemies[min_idx].pose.position.z;
        enemy_chasing_pub->publish(enemy_chasing_msg);

        RCLCPP_INFO(this->get_logger(), "《《《《《《《《《《《《《《 追击英雄");
        RCLCPP_INFO(this->get_logger(), "position x y z: %f, %f, %f",
            response->position.pose.position.x, response->position.pose.position.y, response->position.pose.position.z);
    }
    // 策略3：攻击前哨站
    else if(is_lock[5]) //打前哨站
    {
        response->err_code = 1;
        response->id = "outpost";

        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = false;
        enemy_chasing_pub->publish(enemy_chasing_msg);

        RCLCPP_INFO(this->get_logger(), "打前哨站");
    }
    // 策略4：攻击基地
    else if(is_lock[6]) //打基地
    {
        response->err_code = 1;
        response->id = "base";

        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = false;
        enemy_chasing_pub->publish(enemy_chasing_msg);

        RCLCPP_INFO(this->get_logger(), "打基地");
    }
    // 策略5：在被识别的车中按照选车优先级发
    else
    {
        int i = 0;
        int aim_index = string_id2int_id(aim_priority[i]);
        while(!enemies->enemies[aim_index].is_detected)
        {
            i++;
            aim_index = string_id2int_id(aim_priority[i]);
            if(i >= 5) { // 理论上不会进到这种情况，但还是保留
                response->err_code = 0;
                rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
                enemy_chasing_msg.is_in_chase = false;
                enemy_chasing_pub->publish(enemy_chasing_msg);

                RCLCPP_INFO(this->get_logger(), "无特殊情况，默认听自瞄的");
                return;
            }
        }
        response->err_code = 1;
        response->id = enemies->enemies[aim_index].id;
        response->position.is_detected = enemies->enemies[aim_index].is_detected;
        response->position.occlusion = enemies->enemies[aim_index].occlusion;
        response->position.invincible = received_autoaim_strategy->enemy_invincible[aim_index];
        response->position.main = enemies->enemies[aim_index].main;
        response->position.id = enemies->enemies[aim_index].id;
        response->position.hp = received_autoaim_strategy->enemy_blood[aim_index];
        response->position.pose.position.x = enemies->enemies[aim_index].pose.position.x;
        response->position.pose.position.y = enemies->enemies[aim_index].pose.position.y;
        response->position.pose.position.z = enemies->enemies[aim_index].pose.position.z;
        response->position.pose.orientation.x = enemies->enemies[aim_index].pose.orientation.x;
        response->position.pose.orientation.y = enemies->enemies[aim_index].pose.orientation.y;
        response->position.pose.orientation.z = enemies->enemies[aim_index].pose.orientation.z;
        response->position.pose.orientation.w = enemies->enemies[aim_index].pose.orientation.w;
        response->position.attack_enhance = enemies->enemies[aim_index].attack_enhance;
        RCLCPP_INFO(this->get_logger(), "按选车优先级选择id: %s", response->id.c_str());

        if (response->position.occlusion) {
            omni_detected = true;
            omni_detected_id_index = aim_index;
        }

        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = if_default_chasing; //接入yaml文件中
        enemy_chasing_msg.aim_goal.pose.position.x = enemies->enemies[aim_index].pose.position.x;
        enemy_chasing_msg.aim_goal.pose.position.y = enemies->enemies[aim_index].pose.position.y;
        enemy_chasing_msg.aim_goal.pose.position.z = enemies->enemies[aim_index].pose.position.z;
        enemy_chasing_msg.aim_goal.pose.orientation.x = enemies->enemies[aim_index].pose.orientation.x;
        enemy_chasing_msg.aim_goal.pose.orientation.y = enemies->enemies[aim_index].pose.orientation.y;
        enemy_chasing_msg.aim_goal.pose.orientation.z = enemies->enemies[aim_index].pose.orientation.z;
        enemy_chasing_msg.aim_goal.pose.orientation.w = enemies->enemies[aim_index].pose.orientation.w;
        enemy_chasing_pub->publish(enemy_chasing_msg);
    }
}

/**
 * @brief 自瞄策略信息回调函数
 * @param msg 自瞄策略消息
 * @details 接收并存储自瞄策略信息，包括白名单、敌人血量和无敌状态
 */
void SentryStrategyService::AutoaimStrategyCallback(rm_interfaces::msg::AutoaimStrategy::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "receive decision info");
    for(int i = 0; i < 7; i++)
    {
        received_autoaim_strategy->whitelist[i] = msg->whitelist[i];
        received_autoaim_strategy->enemy_blood[i] = 200; //msg->enemy_blood[i];
        received_autoaim_strategy->enemy_invincible[i] = false; //msg->enemy_invincible[i]; //DEBUG 上场前开
    }
    dog_cnt++;
    if(dog_cnt >= dog_cnt_threshold)
    {
        rm_interfaces::msg::EnemyChasing enemy_chasing_msg;
        enemy_chasing_msg.is_in_chase = false;
        enemy_chasing_pub->publish(enemy_chasing_msg);
        RCLCPP_INFO(this->get_logger(), "长时间未收到自瞄信息，退出追击模式");
    }
}

/**
 * @brief 全向感知信息回调函数
 * @param msg 全向感知消息
 * @details 接收并存储全向感知信息，用于补充自瞄检测不到的敌人信息
 */
void SentryStrategyService::OcclusionCallback(rm_interfaces::msg::EnemyPositions::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "----------------全向感知---------------");

    if (if_in_lost_waiting) {
        lost_delay_cnt++;
        if (lost_delay_cnt >= 12) {
            RCLCPP_INFO(this->get_logger(), "丢失等待时间结束！");
            if_in_lost_waiting = false;
            omni_detected = false;
        }
    }

    /* 获取全向感知信息并存储 */
    int occlusion_size = msg->enemies.size();
    if (occlusion_size != 5)
    {
        RCLCPP_WARN(this->get_logger(), "全向感知数量有误");
        return;
    }
    for (int i = 0; i < occlusion_size; i++)
    {
        //根据当前string_id转为int格式，按顺序存入occlusion_detector中
        int occlusion_id_index = string_id2int_id(msg->enemies[i].id);
        if (occlusion_id_index == -1)
        {
            RCLCPP_WARN(this->get_logger(), "全向感知获取的string_id信息有误");
            return;
        }
        occlusion_detector->enemies[occlusion_id_index].is_detected = msg->enemies[i].is_detected;
        occlusion_detector->enemies[occlusion_id_index].occlusion = msg->enemies[i].occlusion;
        occlusion_detector->enemies[occlusion_id_index].main = msg->enemies[i].main;
        occlusion_detector->enemies[occlusion_id_index].id = msg->enemies[i].id;
        occlusion_detector->enemies[occlusion_id_index].pose = msg->enemies[i].pose;
        occlusion_detector->enemies[occlusion_id_index].attack_enhance = msg->enemies[i].attack_enhance;
        occlusion_detector->enemies[occlusion_id_index].pose.position.x = msg->enemies[i].pose.position.x;
        occlusion_detector->enemies[occlusion_id_index].pose.position.y = msg->enemies[i].pose.position.y;
        occlusion_detector->enemies[occlusion_id_index].pose.position.z = msg->enemies[i].pose.position.z;
        occlusion_detector->enemies[occlusion_id_index].pose.orientation.x = msg->enemies[i].pose.orientation.x;
        occlusion_detector->enemies[occlusion_id_index].pose.orientation.y = msg->enemies[i].pose.orientation.y;
        occlusion_detector->enemies[occlusion_id_index].pose.orientation.z = msg->enemies[i].pose.orientation.z;
        occlusion_detector->enemies[occlusion_id_index].pose.orientation.w = msg->enemies[i].pose.orientation.w;

        if (occlusion_detector->enemies[occlusion_id_index].is_detected) {
            RCLCPP_INFO(this->get_logger(), "occlusion detect id: %s",
                occlusion_detector->enemies[occlusion_id_index].id.c_str());
        }

    }
}
