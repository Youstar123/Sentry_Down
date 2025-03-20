#include <string.h>
#include "trans_task.h"
#include "rm_module.h"
#include "rm_config.h"
#include "robot.h"
#include "usbd_cdc_if.h"
#include "can.h"
/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(referee_topic);
static McnNode_t referee_topic_node;
static struct referee_msg referee_data;
// 发布
MCN_DECLARE(trans_fdb);
static struct trans_fdb_msg trans_fdb_data;
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);
/*------------------------------传输数据相关 --------------------------------- */
int recv_flag=0;
FrameTypeDef upper_rx_data;       //接收上位机数据中转帧
RpyTypeDef rpy_rx_data;  //接收欧拉角方式控制数据帧
RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = GIMBAL,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
OdomTypeDef odom_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = CHASSIS_ODOM,
        .LEN = FRAME_ODOM_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
static uint16_t friendly_1_robot_HP = 0, friendly_3_robot_HP = 0,friendly_4_robot_HP = 0, friendly_7_robot_HP = 0, enemy_1_robot_HP = 0,enemy_3_robot_HP = 0,enemy_4_robot_HP = 0,enemy_7_robot_HP = 0;
// dji 电机同一组id的can控制帧
struct dji_msg
{
    uint32_t id;
    uint8_t data[8];
};
static struct dji_msg data_send;
static struct dji_msg send_msg_gimbal[6] = {
        [0] = {0x3ff},
        [1] = {0x300},
        [2] = {0x310},
        [3] = {0x320},
        [4] = {0x330},
        [5] = {0x340},
};
/* ------------------------------- 发送数据指令 ------------------------------ */
static void send_data(OdomTypeDef data_o);
static void pack_odom(OdomTypeDef *frame, float data1, float data2,float data3,float data4,float data5,float data6,float data7,float data8,float data9,float data10,float data11,float data12,float data13,float data14,float data15,float data16,float data17,float data18,float data19,float data20,float data21);
static void check_odom(OdomTypeDef *frame);
static void pack_rpy(RpyTypeDef *frame, float yaw, float pitch,float roll);
static void check_rpy(RpyTypeDef *frame);
extern void CAN_send(CAN_HandleTypeDef *can, uint32_t send_id, uint8_t data[]);
static void Can_send_gimbal(float data1_original,float data2_original, uint32_t send_id);
static void Can_send_gimbal2(uint16_t data1, uint16_t data2, uint8_t data3, uint8_t data4, uint16_t data5, uint32_t send_id);
static void Can_send_gimbal3(float data1, uint16_t data2, uint8_t data3, uint8_t data4, uint32_t send_id);
/* ------------------------------- 接受数据指令 ------------------------------ */
static void get_data(void);
/* -------------------------------- trans线程主体 -------------------------------- */
void trans_task_init(void)
{
    trans_sub_init();
}

void trans_control_task(void)
{
    trans_sub_pull();
    send_data(odom_tx_data);
    get_data();
    trans_pub_push();
}
/**
 * @brief 向上位机发送数据
 */
static void send_data(OdomTypeDef data_o)
{
// 根据 robot_id 选择友方和敌方的血量
    if ((float)referee_data.robot_status.robot_id < 100) {
        // 红方机器人
        friendly_1_robot_HP = referee_data.game_robot_HP_t.red_1_robot_HP;
        friendly_3_robot_HP = referee_data.game_robot_HP_t.red_3_robot_HP;
        friendly_4_robot_HP = referee_data.game_robot_HP_t.red_4_robot_HP;
        friendly_7_robot_HP = referee_data.game_robot_HP_t.red_7_robot_HP;
        // 蓝方机器人
        enemy_1_robot_HP = referee_data.game_robot_HP_t.blue_1_robot_HP;
        enemy_3_robot_HP = referee_data.game_robot_HP_t.blue_3_robot_HP;
        enemy_4_robot_HP = referee_data.game_robot_HP_t.blue_4_robot_HP;
        enemy_7_robot_HP = referee_data.game_robot_HP_t.blue_7_robot_HP;
    } else {
        // 蓝方机器人
        friendly_1_robot_HP= referee_data.game_robot_HP_t.blue_1_robot_HP;
        friendly_3_robot_HP= referee_data.game_robot_HP_t.blue_3_robot_HP;
        friendly_4_robot_HP= referee_data.game_robot_HP_t.blue_4_robot_HP;
        friendly_7_robot_HP= referee_data.game_robot_HP_t.blue_7_robot_HP;
        // 红方机器人
        enemy_1_robot_HP= referee_data.game_robot_HP_t.red_1_robot_HP;
        enemy_3_robot_HP= referee_data.game_robot_HP_t.red_3_robot_HP;
        enemy_4_robot_HP= referee_data.game_robot_HP_t.red_4_robot_HP;
        enemy_7_robot_HP= referee_data.game_robot_HP_t.red_7_robot_HP;
    }
/*填充数据*/
    pack_odom(&data_o, (float)(((referee_data.field_event.event_type) >> 21) & 0x03),
              (float)(((referee_data.field_event.event_type) >> 2) & 0x01),
              (float)referee_data.game_status.game_progress,
              (float)referee_data.robot_status.current_HP,
              (float)referee_data.bullet_remaining_t.bullet_remaining_num_17mm,
              (float)(referee_data.power_heat_data_t.shooter_id1_17mm_cooling_heat + referee_data.power_heat_data_t.shooter_id2_17mm_cooling_heat),
              (float)friendly_1_robot_HP,
              (float)friendly_3_robot_HP,
              (float)friendly_4_robot_HP,
              (float)friendly_7_robot_HP,
              (float)enemy_1_robot_HP,
              (float)enemy_3_robot_HP,
              (float)enemy_4_robot_HP,
              (float)enemy_7_robot_HP,
              (float)referee_data.game_status.stage_remain_time,
              (float)referee_data.robot_hurt_t.armor_id,
              17.0f,18.0f,19.0f,20.0f,21.0f);
    CDC_Transmit_FS((uint8_t*)&data_o, sizeof(data_o));

    Can_send_gimbal(ins.gyro[2],ins.yaw_total_angle,send_msg_gimbal[0].id);
    Can_send_gimbal2(referee_data.robot_status.chassis_power_limit,referee_data.power_heat_data_t.buffer_energy,(uint8_t)trans_fdb_data.linear_z,referee_data.game_status.game_progress,(referee_data.power_heat_data_t.shooter_id1_17mm_cooling_heat + referee_data.power_heat_data_t.shooter_id2_17mm_cooling_heat),send_msg_gimbal[1].id);
    Can_send_gimbal3(trans_fdb_data.angular_z,referee_data.robot_status.shooter_barrel_heat_limit,referee_data.robot_hurt_t.armor_id,referee_data.robot_hurt_t.hurt_type,send_msg_gimbal[2].id);
    Can_send_gimbal(trans_fdb_data.linear_y,trans_fdb_data.linear_x,send_msg_gimbal[3].id);
   }

static void pack_odom(OdomTypeDef *frame, float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12, float data13, float data14, float data15, float data16, float data17, float data18, float data19, float data20, float data21) {
    int8_t odom_tx_buffer[FRAME_ODOM_LEN] = {0};
    int32_t odom_data1 = 0;
    uint32_t *odom_data2 = (uint32_t *)&odom_data1;
    float data[21] = {data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12, data13, data14, data15, data16, data17, data18, data19, data20, data21};

    for (int i = 0; i < 21; i++) {
        odom_data1 = (int32_t)(data[i] * 10000);  // Scale the data
        int index = i * 4;  // Calculate the starting index for each data chunk
        odom_tx_buffer[index]     = *odom_data2;
        odom_tx_buffer[index + 1] = *odom_data2 >> 8;
        odom_tx_buffer[index + 2] = *odom_data2 >> 16;
        odom_tx_buffer[index + 3] = *odom_data2 >> 24;
    }

    memcpy(&frame->DATA[0], odom_tx_buffer, 84);
    frame->LEN = FRAME_ODOM_LEN;
}

static void check_odom(OdomTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}

static void pack_rpy(RpyTypeDef *frame, float yaw, float pitch,float roll)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    rpy_tx_buffer[0] = 0;
    rpy_data = yaw * 1000;
    rpy_tx_buffer[1] = *gimbal_rpy;
    rpy_tx_buffer[2] = *gimbal_rpy >> 8;
    rpy_tx_buffer[3] = *gimbal_rpy >> 16;
    rpy_tx_buffer[4] = *gimbal_rpy >> 24;
    rpy_data = pitch * 1000;
    rpy_tx_buffer[5] = *gimbal_rpy;
    rpy_tx_buffer[6] = *gimbal_rpy >> 8;
    rpy_tx_buffer[7] = *gimbal_rpy >> 16;
    rpy_tx_buffer[8] = *gimbal_rpy >> 24;
    rpy_data = roll *1000;
    rpy_tx_buffer[9] = *gimbal_rpy;
    rpy_tx_buffer[10] = *gimbal_rpy >> 8;
    rpy_tx_buffer[11] = *gimbal_rpy >> 16;
    rpy_tx_buffer[12] = *gimbal_rpy >> 24;

    memcpy(&frame->DATA[0], rpy_tx_buffer,13);

    frame->LEN = FRAME_RPY_LEN;
}

static void check_rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}

static void get_data(void)
{
    if(recv_flag) {
//        if (!rpy_rx_data.DATA[0]){//绝对角度控制
//            trans_fdb_data.yaw = *(int32_t*)&rpy_rx_data.DATA[1] / 1000.0;
//            trans_fdb_data.pitch = *(int32_t*)&rpy_rx_data.DATA[5] / 1000.0;
//        }
//        else {     //相对角度控制
//            trans_fdb_data.yaw = (*(int32_t *) &rpy_rx_data.DATA[1] / 1000.0);
//            trans_fdb_data.pitch= (*(int32_t *) &rpy_rx_data.DATA[5] / 1000.0);
//            //send_data(rpy_tx_data);
//        }
        trans_fdb_data.linear_x = (*(int32_t *) &rpy_rx_data.DATA[0] / 10000.0);
        trans_fdb_data.linear_y = (*(int32_t *) &rpy_rx_data.DATA[4] / 10000.0);
        trans_fdb_data.linear_z = (*(int32_t *) &rpy_rx_data.DATA[8] / 10000.0);
        trans_fdb_data.angular_x = (*(int32_t *) &rpy_rx_data.DATA[12] / 10000.0);
        trans_fdb_data.angular_y = (*(int32_t *) &rpy_rx_data.DATA[16] / 10000.0);
        trans_fdb_data.angular_z = (*(int32_t *) &rpy_rx_data.DATA[20] / 10000.0);
        recv_flag = 0;
        trans_fdb_data.raw_linear_x = trans_fdb_data.linear_x;
        trans_fdb_data.raw_linear_y = trans_fdb_data.linear_y;
        if(trans_fdb_data.linear_z==1){
            trans_fdb_data.linear_x = 0;
            trans_fdb_data.linear_y = 0;
            trans_fdb_data.angular_z =0;
        }
    }
}

static void Can_send_gimbal(float data1_original,float data2_original, uint32_t send_id){

    uint32_t *temp_data1;
    uint32_t *temp_data2;

    temp_data1= (uint32_t *)&data1_original;
    temp_data2= (uint32_t *)&data2_original;

    data_send.data[3] = *temp_data1 >> 24;
    data_send.data[2] = *temp_data1 >> 16;
    data_send.data[1] = *temp_data1 >> 8;
    data_send.data[0] = *temp_data1;
    data_send.data[7] = *temp_data2 >> 24;
    data_send.data[6] = *temp_data2 >> 16;
    data_send.data[5] = *temp_data2 >> 8;
    data_send.data[4] = *temp_data2;

    CAN_send(&hcan2, send_id, data_send.data); // 发送报文
}
static void Can_send_gimbal2(uint16_t data1, uint16_t data2, uint8_t data3, uint8_t data4, uint16_t data5, uint32_t send_id) {
    // 将数据打包到 data_send.data 数组中
    data_send.data[0] = (data1 >> 8) & 0xFF;  // data1 的高字节
    data_send.data[1] = data1 & 0xFF;         // data1 的低字节

    data_send.data[2] = (data2 >> 8) & 0xFF;  // data2 的高字节
    data_send.data[3] = data2 & 0xFF;         // data2 的低字节

    data_send.data[4] = data3;                // data3（uint8_t）
    data_send.data[5] = data4;                // data4（uint8_t）

    data_send.data[6] = (data5 >> 8) & 0xFF;  // data5 的高字节
    data_send.data[7] = data5 & 0xFF;         // data5 的低字节

    // 发送 CAN 报文
    CAN_send(&hcan2, send_id, data_send.data);
}
static void Can_send_gimbal3(float data1, uint16_t data2, uint8_t data3, uint8_t data4, uint32_t send_id) {
    // 将 float 类型数据 data1 打包到 data_send.data 数组中
    uint32_t *temp_data1 = (uint32_t *) &data1;  // 强制类型转换为 uint32_t
    data_send.data[3] = (*temp_data1 >> 24) & 0xFF;  // 最高字节
    data_send.data[2] = (*temp_data1 >> 16) & 0xFF;  // 次高字节
    data_send.data[1] = (*temp_data1 >> 8) & 0xFF;   // 次低字节
    data_send.data[0] = *temp_data1 & 0xFF;          // 最低字节
    // 将 uint16_t 类型数据 data2 打包到 data_send.data 数组中
    data_send.data[4] = (data2 >> 8) & 0xFF;  // 高字节
    data_send.data[5] = data2 & 0xFF;         // 低字节
    // 将 uint8_t 类型数据 data3 和 data4 打包到 data_send.data 数组中
    data_send.data[6] = data3;  // data3
    data_send.data[7] = data4;  // data4
    // 发送 CAN 报文
    CAN_send(&hcan2, send_id, data_send.data);
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void trans_pub_push(void)
{
    mcn_publish(MCN_HUB(trans_fdb), &trans_fdb_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void trans_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    referee_topic_node = mcn_subscribe(MCN_HUB(referee_topic), NULL, NULL);
}

/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void trans_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
    if (mcn_poll(referee_topic_node))
    {
        mcn_copy(MCN_HUB(referee_topic), referee_topic_node, &referee_data);
    }
}