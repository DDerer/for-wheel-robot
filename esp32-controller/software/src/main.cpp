/* main.c
 * 主代码文件，包含所有主要代码逻辑
 * by L.B.W 2023
 * */

#include <Arduino.h>
#include <PID.h>
#include "driver/twai.h"
#include "board_config.h"
#include "jy901s.h"
#include "leg_pos.h"
#include "lqr_k.h"
#include "leg_conv.h"
#include "leg_spd.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <debug.h>

static_assert(configTICK_RATE_HZ == 1000, "The legacy controller requires 1 ms RTOS ticks");

/*******************宏定义*******************/

//蓝牙各UUID及设备名称
#define UUID_SERVICE "4c9a0001-fb2e-432e-92ec-c6f5316b4689" //服务UUID
#define UUID_RX_CHARA "4c9a0002-fb2e-432e-92ec-c6f5316b4689" //接收特征UUID
#define UUID_TX_CHARA "4c9a0003-fb2e-432e-92ec-c6f5316b4689" //发送特征UUID
#define BLE_NAME "BalanceBot"

//计时工具，计算运行周期或经过时间
uint32_t timeCnt[6] = {0}, lastTimeCnt[6] = {0}, passTimeCnt[6] = {0};
#define TIME_COUNTER(num) do{timeCnt[num] = micros()-lastTimeCnt[num]; lastTimeCnt[num] = micros();}while(0)
#define PASS_TIME_START(num) uint32_t passTimeStart##num = micros()
#define PASS_TIME_STOP(num) passTimeCnt[num] = micros() - passTimeStart##num

/*******************结构体类型和全局变量定义*******************/

//IMU对象及数据
Jy901s imu;
struct IMUData
{
	float yaw, pitch, roll;			 // rad
	float yawSpd, pitchSpd, rollSpd; // rad/s
	float zAccel; // m/s^2
} imuData;

//电机结构体
//leftJoint[0]:左前关节电机, leftJoint[1]:左后关节电机, leftWheel:左车轮电机
//rightJoint[0]:右前关节电机, rightJoint[1]:右后关节电机, rightWheel:右车轮电机
struct Motor
{
	float speed;			   // rad/s
	float angle, offsetAngle;  // rad
	float voltage, maxVoltage; // V
	float torque, torqueRatio; // Nm, voltage = torque / torqueRatio
	float dir;				   // 1 or -1
	float (*calcRevVolt)(float speed); // 指向反电动势计算函数
} leftJoint[2], rightJoint[2], leftWheel, rightWheel; //六个电机对象

//腿部姿态结构体
struct LegPos
{
	float angle, length;   // rad, m
	float dAngle, dLength; // rad/s, m/s
	float ddLength;		   // m/s^2
} leftLegPos, rightLegPos; //左右腿部姿态

//状态变量结构体
struct StateVar
{
	float theta, dTheta;
	float x, dx;
	float phi, dPhi;
} stateVar;

//目标量结构体
struct Target
{
	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float legLength; // m
} target = {0, 0, 0, 0, 0, 0, 0.07f};

//触地检测数据结构体
struct GroundDetector
{
	float leftSupportForce, rightSupportForce;
	bool isTouchingGround, isCuchioning;
} groundDetector = {10, 10, true, false};

//站立过程状态枚举量
enum StandupState {
	StandupState_None,
	StandupState_Prepare,
	StandupState_Standup,
} standupState = StandupState_None;

CascadePID legAnglePID, legLengthPID; //腿部角度和长度控制PID
CascadePID yawPID, rollPID; //机身yaw和roll控制PID

//蓝牙相关全局对象指针
BLECharacteristic *chara_tx, *chara_rx;
BLEServer *server;
BLEService *service;

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效
float sourceVoltage = BoardConfig::kNominalVoltage; //当前电源电压

portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;
Motor *motorList[] = {&leftJoint[0], &leftJoint[1], &leftWheel,
                      &rightJoint[0], &rightJoint[1], &rightWheel};
uint32_t motorReceivedAt[6] = {};
bool motorReceived[6] = {};
volatile bool motorsArmed = false;
volatile bool canReady = false;
volatile bool controlReady = false;
volatile bool standupRequested = false;
volatile bool stopRequested = false;
TaskHandle_t standupTask = nullptr;
uint32_t canRxFrames = 0, canTxErrors = 0;
QueueHandle_t commandQueue;
struct MotionCommand { float speed, yawSpeed, legLength; };
QueueHandle_t motionQueue;

enum class UserCommand { Arm, Stop, Standup, Heading, ConfigureImu };
void Control_Stop();
bool Control_Arm();
bool Control_FeedbackReady();
void Print_Status();
void Handle_Command(UserCommand command);

/*******************函数声明*******************/

void setup();
void loop();

void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir, float (*calcRevVolt)(float speed));
float Motor_CalcRevVolt4010(float speed);
float Motor_CalcRevVolt2804(float speed);
void Motor_InitAll();
void Motor_Update(Motor *motor, uint8_t *data);
void Motor_SetTorque(Motor *motor, float torque);
void Motor_UpdateVoltage(Motor *motor);
void Motor_SendTask(void *arg);

void CAN_RecvCallback(uint32_t id, uint8_t *data);
void CAN_RecvTask(void *arg);
void CAN_Init();
void CAN_SendFrame(uint32_t id, uint8_t *data);

void IMU_Task(void *arg);
void IMU_Init();

void Ctrl_TargetUpdateTask(void *arg);
void LegPos_UpdateTask(void *arg);
void Ctrl_StandupPrepareTask(void *arg);
void Ctrl_Task(void *arg);
void Ctrl_Init();

void BT_SendSampleTask(void *arg);
void BT_Init();

void Serial_Task(void *pvParameters);
void Serial_Init();

void ADC_Task(void *pvParameters);
void ADC_Init();

/*******************各模块函数定义*******************/

/******* 电机模块 *******/

//初始化一个电机对象
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir, float (*calcRevVolt)(float speed))
{
	motor->speed = motor->angle = motor->voltage = motor->torque = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
	motor->calcRevVolt = calcRevVolt;
}

//4010电机反电动势计算函数(输入速度，输出反电动势)
//测量并拟合出不同电压下对应的电机空载转速，调换自变量和因变量就是本函数
//由于该测量方法忽略阻力对空载转速的影响，最终抵消反电动势时也会抵消大部分电机本身的阻力
float Motor_CalcRevVolt4010(float speed)
{
	return 0.00008f * speed * speed * speed - 0.0035f * speed * speed + 0.2322f * speed;
}

//2804电机反电动势计算函数(输入速度，输出反电动势)，测量方法同上
float Motor_CalcRevVolt2804(float speed)
{
	return 0.000004f * speed * speed * speed - 0.0003f * speed * speed + 0.0266f * speed;
}

//初始化所有电机对象
//各个参数需要通过实际测量或拟合得到
void Motor_InitAll()
{
	Motor_Init(&leftJoint[0], 1.431, 7, 0.0316f, -1, Motor_CalcRevVolt4010);
	Motor_Init(&leftJoint[1], -7.76, 7, 0.0317f, 1, Motor_CalcRevVolt4010);
	Motor_Init(&leftWheel, 0, 4.0f, 0.0096f, 1, Motor_CalcRevVolt2804);
	Motor_Init(&rightJoint[0], 0.343, 7, 0.0299f, -1, Motor_CalcRevVolt4010);
	Motor_Init(&rightJoint[1], -2.446, 7, 0.0321f, -1, Motor_CalcRevVolt4010);
	Motor_Init(&rightWheel, 0, 4.0f, 0.0101f, 1, Motor_CalcRevVolt2804);
	// Sending starts after control initialization in setup().
}

//从CAN总线接收到的数据中解析出电机角度和速度
void Motor_Update(Motor *motor, uint8_t *data)
{
    int32_t angle;
    int16_t speed;
    memcpy(&angle, data, sizeof(angle));
    memcpy(&speed, data + 4, sizeof(speed));
    motor->angle = (angle / 1000.0f - motor->offsetAngle) * motor->dir;
    motor->speed = speed / 10.0f * 2.0f * M_PI / 60.0f * motor->dir;
}

//设置电机扭矩
void Motor_SetTorque(Motor *motor, float torque)
{
	portENTER_CRITICAL(&motorMux);
    motor->torque = isfinite(torque) ? torque : 0.0f;
    portEXIT_CRITICAL(&motorMux);
}

//由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
//补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
void Motor_UpdateVoltage(Motor *motor)
{
	float voltage = motor->torque / motor->torqueRatio * motorOutRatio;
	if (motor->speed >= 0)
		voltage += motor->calcRevVolt(motor->speed);
	else if (motor->speed < 0)
		voltage -= motor->calcRevVolt(-motor->speed);
	if (voltage > motor->maxVoltage)
		voltage = motor->maxVoltage;
	else if (voltage < -motor->maxVoltage)
		voltage = -motor->maxVoltage;
	motor->voltage = voltage * motor->dir;
}

//电机指令发送任务
void Motor_SendTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        if (stopRequested) {
            stopRequested = false;
            Control_Stop();
        }
        if (motorsArmed && !Control_FeedbackReady()) {
            Control_Stop();
            Serial.println("DISARMED: IMU/CAN feedback unavailable or too slow");
        }
        uint8_t groups[2][8] = {};
        portENTER_CRITICAL(&motorMux);
        const bool enabled = motorsArmed && controlReady;
        for (size_t i = 0; i < 6; i++) {
            Motor *motor = motorList[i];
            motor->voltage = 0;
            if (enabled) Motor_UpdateVoltage(motor);
            if (!isfinite(motor->voltage)) motor->voltage = 0;
            const int16_t millivolts = static_cast<int16_t>(motor->voltage * 1000.0f);
            const uint8_t slot = BoardConfig::kMotorIds[i] - 1;
            memcpy(&groups[slot / 4][(slot % 4) * 2], &millivolts, sizeof(millivolts));
        }
        portEXIT_CRITICAL(&motorMux);
        if (canReady) {
            CAN_SendFrame(0x100, groups[0]);
            CAN_SendFrame(0x200, groups[1]);
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
    }
}

/******* CAN通信模块 *******/

//CAN收到数据后进入的回调函数
void CAN_RecvCallback(uint32_t id, uint8_t *data)
{
    for (size_t i = 0; i < 6; i++) {
        if (id == 0x100u + BoardConfig::kMotorIds[i]) {
            portENTER_CRITICAL(&motorMux);
            Motor_Update(motorList[i], data);
            motorReceivedAt[i] = millis();
            motorReceived[i] = true;
            canRxFrames++;
            portEXIT_CRITICAL(&motorMux);
            return;
        }
    }
}

//CAN数据帧轮询接收任务
void CAN_RecvTask(void *arg)
{
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK &&
            !msg.extd && !msg.rtr && msg.data_length_code == 8) {
            CAN_RecvCallback(msg.identifier, msg.data);
        }
        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK) continue;
        if (status.state == TWAI_STATE_BUS_OFF) {
            Control_Stop();
            twai_initiate_recovery();
        } else if (status.state == TWAI_STATE_STOPPED) {
            twai_start();
        }
    }
}

//CAN通信外设初始化
void CAN_Init()
{
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(BoardConfig::kCanTx),
        static_cast<gpio_num_t>(BoardConfig::kCanRx), TWAI_MODE_NORMAL);
    general.tx_queue_len = 2;
    general.rx_queue_len = 64;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t result = twai_driver_install(&general, &timing, &filter);
    if (result != ESP_OK) {
        Serial.printf("CAN install failed: %s\n", esp_err_to_name(result));
        return;
    }
    result = twai_start();
    if (result != ESP_OK) {
        twai_driver_uninstall();
        Serial.printf("CAN start failed: %s\n", esp_err_to_name(result));
        return;
    }
    canReady = true;
    xTaskCreatePinnedToCore(CAN_RecvTask, "CAN_RecvTask", 3072, nullptr, 5, nullptr, 1);
}

//发送一帧CAN数据(data为8字节数据)
void CAN_SendFrame(uint32_t id, uint8_t *data)
{
	twai_message_t msg;
	msg.flags = 0;
	msg.identifier = id;
	msg.data_length_code = 8;
	memcpy(msg.data, data, 8);
	// Single shot bounds queued commands when no node acknowledges the bus.
    msg.ss = 1;
    if (twai_transmit(&msg, 0) != ESP_OK) canTxErrors++;
}

/******* 陀螺仪模块 *******/

//陀螺仪数据获取任务
void IMU_Task(void *arg)
{
    while (true) {
        imu.poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void IMU_Init()
{
    imu.begin(BoardConfig::kImuRx, BoardConfig::kImuTx);
    xTaskCreatePinnedToCore(IMU_Task, "IMU_Task", 4096, nullptr, 5, nullptr, 1);
}

/******* 运动控制模块 *******/

//目标量更新任务(根据蓝牙收到的目标量计算实际控制算法的给定量)
void Ctrl_TargetUpdateTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.003f;
	while (1)
	{
		//根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
		if (!motorsArmed) {
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
			continue;
		}
        MotionCommand command;
        if (xQueueReceive(motionQueue, &command, 0) == pdTRUE) {
            target.speedCmd = command.speed;
            target.yawSpeedCmd = command.yawSpeed;
            target.legLength = command.legLength;
        }
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		speedSlopeStep = -(legLength - 0.07f) * 0.02f + 0.002f;

		//计算速度斜坡，斜坡值更新到target.speed
		if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			target.speed = target.speedCmd;
		else
		{
			if(target.speedCmd - target.speed > 0)
				target.speed += speedSlopeStep;
			else
				target.speed -= speedSlopeStep;
		}

		//计算位置目标，并限制在当前位置的±0.1m内
		target.position += target.speed * 0.004f;
		if(target.position - stateVar.x > 0.1f)
			target.position = stateVar.x + 0.1f; 
		else if(target.position - stateVar.x < -0.1f)
			target.position = stateVar.x - 0.1f;

		//限制速度目标在当前速度的±0.3m/s内
		if(target.speed - stateVar.dx > 0.3f)
			target.speed = stateVar.dx + 0.3f;
		else if(target.speed - stateVar.dx < -0.3f)
			target.speed = stateVar.dx - 0.3f;

		//计算yaw方位角目标
		target.yawAngle += target.yawSpeedCmd * 0.004f;
		
		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}

//腿部姿态更新任务(根据关节电机数据计算腿部姿态)
void LegPos_UpdateTask(void *arg)
{
	const float lpfRatio = 0.5f; //低通滤波系数(新值的权重)
	float lastLeftDLength = 0, lastRightDLength = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		float legPos[2], legSpd[2];

		//计算左腿位置
		leg_pos(leftJoint[1].angle, leftJoint[0].angle, legPos);
		leftLegPos.length = legPos[0];
		leftLegPos.angle = legPos[1];

		//计算左腿速度
		leg_spd(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
		leftLegPos.dLength = legSpd[0];
		leftLegPos.dAngle = legSpd[1];

		//计算左腿腿长加速度
		leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
		lastLeftDLength = leftLegPos.dLength;

		//计算右腿位置
		leg_pos(rightJoint[1].angle, rightJoint[0].angle, legPos);
		rightLegPos.length = legPos[0];
		rightLegPos.angle = legPos[1];

		//计算右腿速度
		leg_spd(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
		rightLegPos.dLength = legSpd[0];
		rightLegPos.dAngle = legSpd[1];

		//计算右腿腿长加速度
		rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
		lastRightDLength = rightLegPos.dLength;

		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}

//站立准备任务(将机器人从任意姿态调整到准备站立前的劈叉状态)
void Ctrl_StandupPrepareTask(void *arg)
{
	standupState = StandupState_Prepare;

	//将左腿向后摆
	Motor_SetTorque(&leftJoint[0], 0.2f);
	Motor_SetTorque(&leftJoint[1], 0.2f);
    const uint32_t leftStartedAt = millis();
    while (leftLegPos.angle < M_3PI_4) {
        if (!motorsArmed || millis() - leftStartedAt > BoardConfig::kStandupTimeoutMs) {
            Control_Stop();
            standupTask = nullptr;
            vTaskDelete(nullptr);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
	Motor_SetTorque(&leftJoint[0], 0);
	Motor_SetTorque(&leftJoint[1], 0);
	vTaskDelay(1000);

    if (!motorsArmed) {
        standupTask = nullptr;
        vTaskDelete(nullptr);
    }
	//将右腿向前摆
	Motor_SetTorque(&rightJoint[0], -0.2f);
	Motor_SetTorque(&rightJoint[1], -0.2f);
    const uint32_t rightStartedAt = millis();
    while (rightLegPos.angle > M_PI_4) {
        if (!motorsArmed || millis() - rightStartedAt > BoardConfig::kStandupTimeoutMs) {
            Control_Stop();
            standupTask = nullptr;
            vTaskDelete(nullptr);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
	Motor_SetTorque(&rightJoint[0], 0);
	Motor_SetTorque(&rightJoint[1], 0);
	vTaskDelay(1000);

	//完成准备动作，关闭电机结束任务
	Motor_SetTorque(&leftJoint[0], 0);
	Motor_SetTorque(&leftJoint[1], 0);
	Motor_SetTorque(&leftWheel, 0);
	Motor_SetTorque(&rightJoint[0], 0);
	Motor_SetTorque(&rightJoint[1], 0);
	Motor_SetTorque(&rightWheel, 0);
	
    if (motorsArmed) standupState = StandupState_Standup;
    standupTask = nullptr;

	vTaskDelete(NULL);
}

//主控制任务
void Ctrl_Task(void *arg)
{
	const float wheelRadius = 0.026f; //m，车轮半径
	const float legMass = 0.05f; //kg，腿部质量

	TickType_t xLastWakeTime = xTaskGetTickCount();

	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
						{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.07f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;

	while (1)
	{
        const Jy901sSample sample = imu.snapshot();
        imuData = {sample.yaw, sample.pitch, sample.roll,
                   sample.yawSpeed, sample.pitchSpeed, sample.rollSpeed, sample.zAcceleration};
        if (!motorsArmed || !Control_FeedbackReady()) {
            controlReady = false;
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
            continue;
        }
		if (!isfinite(leftLegPos.length) || !isfinite(rightLegPos.length) ||
            leftLegPos.length < 0.04f || rightLegPos.length < 0.04f ||
            leftLegPos.length > 0.15f || rightLegPos.length > 0.15f) {
            Control_Stop();
            continue;
        }
		//计算状态变量
		stateVar.phi = imuData.pitch;
		stateVar.dPhi = imuData.pitchSpd;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imuData.pitch;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imuData.pitchSpd;
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

        if (standupRequested) {
            standupRequested = false;
            standupState = StandupState_Prepare;
            if (xTaskCreatePinnedToCore(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096,
                                       nullptr, 1, &standupTask, 1) != pdPASS) {
                Control_Stop();
            }
        }
		//如果正在站立准备状态，则不进行后续控制
		if(standupState == StandupState_Prepare)
		{
            controlReady = true;
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}

		//计算LQR反馈矩阵
		float kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);
		if(groundDetector.isTouchingGround) //正常触地状态
		{
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 2; j++)
					k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
			}
		}
		else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
		{
			memset(k, 0, sizeof(k));
			k[1][0] = kRes[1] * -2;
			k[1][1] = kRes[3] * -10;
		}

		//准备状态变量
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		//与给定量作差
		x[2] -= target.position;
		x[3] -= target.speed;

		//矩阵相乘，计算LQR输出
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

		//计算yaw轴PID输出
		PID_CascadeCalc(&yawPID, target.yawAngle, imuData.yaw, imuData.yawSpd);
		
		//设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
		if(groundDetector.isTouchingGround) //正常接地状态
		{
			Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
			Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);
		}
		else //腿部离地状态，关闭车轮电机
		{
			Motor_SetTorque(&leftWheel, 0);
			Motor_SetTorque(&rightWheel, 0);
		}

		//根据离地状态修改目标腿长，并计算腿长PID输出
		PID_CascadeCalc(&legLengthPID, (groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? target.legLength : 0.12f, legLength, dLegLength);
		//计算roll轴PID输出
		PID_CascadeCalc(&rollPID, target.rollAngle, imuData.roll, imuData.rollSpd);
		//根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
		float leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6-rollPID.output : 0);
		float rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6+rollPID.output : 0);
		if(leftLegPos.length > 0.12f) //保护腿部不能伸太长
			leftForce -= (leftLegPos.length - 0.12f) * 100;
		if(rightLegPos.length > 0.12f)
			rightForce -= (rightLegPos.length - 0.12f) * 100;
		
		//计算左右腿的地面支持力
		groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - imuData.zAccel);
		groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - imuData.zAccel);
		//更新离地检测器数据
		static uint32_t lastTouchTime = 0;
		bool isTouchingGround = groundDetector.leftSupportForce > 3 && groundDetector.rightSupportForce > 3; //判断当前瞬间是否接地
		if(!isTouchingGround && millis() - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
			isTouchingGround = true;
		if(!groundDetector.isTouchingGround && isTouchingGround) //判断转为接地状态，标记进入缓冲状态
		{
			target.position = stateVar.x;
			groundDetector.isCuchioning = true;
			lastTouchTime = millis();
		}
		if(groundDetector.isCuchioning && legLength < target.legLength) //缓冲状态直到腿长压缩到目标腿长结束
			groundDetector.isCuchioning = false;
		groundDetector.isTouchingGround = isTouchingGround;

		//计算左右腿角度差PID输出
		PID_CascadeCalc(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
		
		//计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
		float leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.07f);
		float rightTp = lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.07f);
		
		//使用VMC计算各关节电机输出扭矩
		float leftJointTorque[2]={0};
		leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
		float rightJointTorque[2]={0};
		leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);
		
		//保护腿部角度不超限
		float leftTheta = leftLegPos.angle - imuData.pitch - M_PI_2;
		float rightTheta = rightLegPos.angle - imuData.pitch - M_PI_2;
		#define PROTECT_CONDITION (leftTheta < -M_PI_4 || leftTheta > M_PI_4 || \
								   rightTheta < -M_PI_4 || rightTheta > M_PI_4 || \
								   imuData.pitch > M_PI_4 || imuData.pitch < -M_PI_4) //腿部角度超限保护条件
        if (PROTECT_CONDITION && standupState == StandupState_None) {
            Control_Stop();
            Serial.println("DISARMED: body or leg angle limit");
            continue;
        }
        if (standupState == StandupState_Standup &&
            (!PROTECT_CONDITION || leftTheta < -M_PI_4 || rightTheta > M_PI_4)) {
            standupState = StandupState_None;
        }
        if (!isfinite(leftJointTorque[0]) || !isfinite(leftJointTorque[1]) ||
            !isfinite(rightJointTorque[0]) || !isfinite(rightJointTorque[1]) ||
            !isfinite(leftWheel.torque) || !isfinite(rightWheel.torque)) {
            Control_Stop();
            continue;
        }

		//设定关节电机输出扭矩
		Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
		Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
		Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
		Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);
        controlReady = motorsArmed;

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
}

//控制模块初始化
void Ctrl_Init()
{
	//初始化各个PID参数
	PID_Init(&yawPID.inner, 0.01, 0, 0, 0, 0.1);
	PID_Init(&yawPID.outer, 10, 0, 0, 0, 2);
	PID_Init(&rollPID.inner, 1, 0, 5, 0, 5);
	PID_Init(&rollPID.outer, 20, 0, 0, 0, 3);
	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
	PID_Init(&legLengthPID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f);
	PID_Init(&legLengthPID.outer, 5.0f, 0, 0.0f, 0.0f, 0.5f);
	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
	PID_Init(&legAnglePID.inner, 0.04, 0, 0, 0, 1);
	PID_Init(&legAnglePID.outer, 12, 0, 0, 0, 20);
	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);

	//触发各个控制任务
	xTaskCreatePinnedToCore(Ctrl_TargetUpdateTask, "Ctrl_TargetUpdateTask", 4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(LegPos_UpdateTask, "LegPos_UpdateTask", 4096, NULL, 2, NULL, 1);
	vTaskDelay(2);
	xTaskCreatePinnedToCore(Ctrl_Task, "Ctrl_Task", 4096, NULL, 1, NULL, 1);
}

/******* 蓝牙模块 *******/

//通过蓝牙notify发送数据
extern "C" void BT_Send(uint8_t *data, uint16_t len)
{
    if (chara_tx == nullptr || server->getConnectedCount() == 0) return;
	chara_tx->setValue(data, len);
	chara_tx->notify();
}

//通过蓝牙发送采样数据的任务
void BT_SendSampleTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t runDuration = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(arg)) * 1000u;
	uint32_t startTime = millis();
	while (1)
	{
		String str = "";
		str += String(target.speedCmd, 3) + "," + String(target.speed, 3) + "," + String(stateVar.dx, 3)  + "," + String(stateVar.phi) + "," + String(stateVar.theta) + "\r\n";
		// str += String(target.yawAngle, 3) + "," + String(imuData.yaw, 3) + "," + String(imuData.yawSpd, 3) + "\r\n";
		BT_Send((uint8_t *)str.c_str(), str.length());

		if (runDuration > 0 && millis() - startTime > runDuration)
			vTaskDelete(NULL);

		vTaskDelayUntil(&xLastWakeTime, 100);
	}
}

//重写蓝牙服务回调类，处理连接和断连事件
class MyBleServerCallbacks : public BLEServerCallbacks
{
public:
	void onConnect(BLEServer *server)
	{
		Serial.println("onConnect");
		server->getAdvertising()->stop();
	}
	void onDisconnect(BLEServer *server)
	{
        stopRequested = true;
		Serial.println("onDisconnect");
		server->getAdvertising()->start();
	}
};

//重写蓝牙特征回调类，处理特征读写事件
class MyBleCharaCallbacks : public BLECharacteristicCallbacks
{
public:
	void onWrite(BLECharacteristic *chara) //特征写入事件，收到蓝牙数据
	{
		uint32_t len = chara->getLength();
		uint8_t *data = chara->getData();

		// Serial.print("onWrite: " + String(len) + " bytes: "); //测试蓝牙收发
		// for (int i = 0; i < len; i++)
		// 	Serial.print(String(data[i], HEX) + " ");
		// Serial.println();
		// BT_Send(data, len);

        if (len == 0) return;
		if(data[0] == 0xAA && len == 4 && motorsArmed) //0xAA开头，表示目标值控制指令
		{
            const MotionCommand command = {
                constrain(data[2] / 100.0f - 1.0f, -1.0f, 1.0f),
                constrain(-(data[1] / 100.0f - 1.0f) * 1.5f, -1.5f, 1.5f),
                constrain(data[3] / 200.0f * 0.02f + 0.07f, 0.07f, 0.09f)
            };
            xQueueOverwrite(motionQueue, &command);
		}
		else if(data[0] == 0xAB) //0xAB开头，表示触发起立过程
		{
            const UserCommand command = UserCommand::Standup;
            xQueueSend(commandQueue, &command, 0);
		}
		else if(data[0] == 0xAC && len == 2) //0xAC开头，表示触发一次数据采样
		{
			xTaskCreatePinnedToCore(BT_SendSampleTask, "BT_SendSampleTask", 4096, reinterpret_cast<void *>(static_cast<uintptr_t>(data[1])), 1, NULL, 1);
		}
		else if(data[0] == 0xDB) //0xDB开头，表示是调试指令
		{
			Debug_SerialRecv(data, len);
		}
	}
};

//蓝牙模块初始化，创建设备、服务、特征并启动广播
void BT_Init()
{
	BLEDevice::init(BLE_NAME);
	server = BLEDevice::createServer();
	server->setCallbacks(new MyBleServerCallbacks());
	service = server->createService(UUID_SERVICE);
	chara_tx = service->createCharacteristic(UUID_TX_CHARA, BLECharacteristic::PROPERTY_NOTIFY);
	chara_tx->addDescriptor(new BLE2902());
	chara_rx = service->createCharacteristic(UUID_RX_CHARA, BLECharacteristic::PROPERTY_WRITE_NR);
	chara_rx->setCallbacks(new MyBleCharaCallbacks());
	service->start();
	server->getAdvertising()->start();
}

/******* 串口模块 *******/

//串口定时发送任务(调试用)
void Serial_Task(void *pvParameters)
{
    uint32_t lastPrint = 0;
    char command[32] = {};
    size_t length = 0;
    while (true) {
        while (Serial.available()) {
            const char value = static_cast<char>(Serial.read());
            if (value == '\n' || value == '\r') {
                if (length > 0) {
                    command[length] = '\0';
                    if (!strcmp(command, "arm")) Handle_Command(UserCommand::Arm);
                    else if (!strcmp(command, "stop")) Handle_Command(UserCommand::Stop);
                    else if (!strcmp(command, "stand")) Handle_Command(UserCommand::Standup);
                    else if (!strcmp(command, "heading")) Handle_Command(UserCommand::Heading);
                    else if (!strcmp(command, "imu-config")) Handle_Command(UserCommand::ConfigureImu);
                    else if (!strcmp(command, "status")) Print_Status();
                    else Serial.println("Commands: status, imu-config, arm, stop, stand, heading");
                    length = 0;
                }
            } else if (length < sizeof(command) - 1) {
                command[length++] = value;
            } else {
                length = 0;
            }
        }
        UserCommand queued;
        while (xQueueReceive(commandQueue, &queued, 0) == pdTRUE) Handle_Command(queued);
        if (millis() - lastPrint >= 1000) {
            lastPrint = millis();
            Print_Status();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//串口模块初始化
void Serial_Init()
{
	xTaskCreatePinnedToCore(Serial_Task, "Serial_Task", 4096, NULL, 1, NULL, 1);
}

// The PCB has power LEDs and no confirmed battery ADC input.
void ADC_Task(void *arg)
{
    while (true) {
        sourceVoltage = analogReadMilliVolts(BoardConfig::kBatteryAdc) / 1000.0f *
                        BoardConfig::kBatteryDividerRatio;
        if (sourceVoltage < 8.0f) Control_Stop();
        motorOutRatio = constrain((12.0f - sourceVoltage) / 10.0f + 0.7f, 0.0f, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void ADC_Init()
{
    sourceVoltage = BoardConfig::kNominalVoltage;
    motorOutRatio = 0.7f;
    if (BoardConfig::kBatteryAdc >= 0) {
        analogSetPinAttenuation(BoardConfig::kBatteryAdc, ADC_11db);
        xTaskCreatePinnedToCore(ADC_Task, "ADC_Task", 2048, nullptr, 2, nullptr, 1);
    }
}

bool Control_FeedbackReady()
{
    const Jy901sSample sample = imu.snapshot();
    const uint32_t now = millis();
    if (!canReady || sample.configuring) return false;
    for (size_t i = 0; i < 3; i++) {
        if (sample.frames[i] == 0 || now - sample.receivedAt[i] > BoardConfig::kImuTimeoutMs ||
            sample.rates[i] < BoardConfig::kMinimumImuRateHz) return false;
    }
    portENTER_CRITICAL(&motorMux);
    bool ready = true;
    const uint32_t motorNow = millis();
    for (size_t i = 0; i < 6; i++) {
        ready = ready && motorReceived[i] && motorNow - motorReceivedAt[i] <= BoardConfig::kFeedbackTimeoutMs;
    }
    portEXIT_CRITICAL(&motorMux);
    twai_status_info_t status;
    return ready && twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_RUNNING;
}

void Control_Stop()
{
    portENTER_CRITICAL(&motorMux);
    motorsArmed = false;
    controlReady = false;
    for (Motor *motor : motorList) motor->torque = motor->voltage = 0;
    portEXIT_CRITICAL(&motorMux);
    standupRequested = false;
    standupState = StandupState_None;
    target.speedCmd = target.speed = target.yawSpeedCmd = 0;
    if (motionQueue != nullptr) xQueueReset(motionQueue);
}

bool Control_Arm()
{
    if (standupTask != nullptr || digitalRead(BoardConfig::kButtonStop) == LOW) {
        Serial.println("ARM rejected: stop button pressed or standup task is finishing");
        return false;
    }
    if (!BoardConfig::kImuMountingConfirmed || !BoardConfig::kMotorCalibrationConfirmed) {
        Serial.println("ARM rejected: confirm mounting and motor calibration in board_config.h");
        return false;
    }
    if (!Control_FeedbackReady()) {
        Serial.println("ARM rejected: need fresh six-motor feedback and >=80Hz IMU data");
        return false;
    }
    if (motorsArmed) return true;
    for (CascadePID *pid : {&yawPID, &rollPID, &legLengthPID, &legAnglePID}) {
        PID_Clear(&pid->inner);
        PID_Clear(&pid->outer);
    }
    target.position = (leftWheel.angle + rightWheel.angle) / 2 * 0.026f;
    target.yawAngle = imu.snapshot().yaw;
    portENTER_CRITICAL(&motorMux);
    for (Motor *motor : motorList) motor->torque = motor->voltage = 0;
    controlReady = false;
    motorsArmed = true;
    portEXIT_CRITICAL(&motorMux);
    Serial.println("ARMED");
    return true;
}

void Handle_Command(UserCommand command)
{
    switch (command) {
        case UserCommand::Arm: Control_Arm(); break;
        case UserCommand::Stop: Control_Stop(); Serial.println("DISARMED"); break;
        case UserCommand::Standup:
            if (motorsArmed && standupState == StandupState_None && standupTask == nullptr) standupRequested = true;
            break;
        case UserCommand::Heading: target.yawAngle = imu.snapshot().yaw; break;
        case UserCommand::ConfigureImu:
            Control_Stop();
            imu.requestConfiguration();
            break;
    }
}

void Print_Status()
{
    const Jy901sSample sample = imu.snapshot();
    Serial.printf("%s IMU baud=%lu Hz=%.1f/%.1f/%.1f RPY(deg)=%.2f,%.2f,%.2f gyro(rad/s)=%.3f,%.3f,%.3f az=%.3f crc=%lu\n",
                  motorsArmed ? "ARMED" : "DISARMED", static_cast<unsigned long>(sample.baud),
                  sample.rates[0], sample.rates[1], sample.rates[2],
                  sample.roll * 180 / PI, sample.pitch * 180 / PI, sample.yaw * 180 / PI,
                  sample.rollSpeed, sample.pitchSpeed, sample.yawSpeed, sample.zAcceleration,
                  static_cast<unsigned long>(sample.checksumErrors));
    const uint32_t now = millis();
    portENTER_CRITICAL(&motorMux);
    uint32_t timestamps[6];
    bool seen[6];
    float angles[6];
    memcpy(timestamps, motorReceivedAt, sizeof(timestamps));
    memcpy(seen, motorReceived, sizeof(seen));
    for (size_t i = 0; i < 6; i++) angles[i] = motorList[i]->angle;
    portEXIT_CRITICAL(&motorMux);
    for (size_t i = 0; i < 6; i++) {
        Serial.printf(" ID%u=%s angle=%.3f age=%lums", BoardConfig::kMotorIds[i],
                      seen[i] && now - timestamps[i] <= BoardConfig::kFeedbackTimeoutMs ? "online" : "offline",
                      angles[i], static_cast<unsigned long>(now - timestamps[i]));
    }
    twai_status_info_t status = {};
    if (canReady && twai_get_status_info(&status) == ESP_OK) {
        Serial.printf(" CAN state=%d TEC=%lu REC=%lu rxMiss=%lu txFail=%lu", status.state,
                      static_cast<unsigned long>(status.tx_error_counter),
                      static_cast<unsigned long>(status.rx_error_counter),
                      static_cast<unsigned long>(status.rx_missed_count),
                      static_cast<unsigned long>(status.tx_failed_count));
    }
    Serial.println();
    Serial.printf("CAN rx=%lu txQueueDrops=%lu Vbat=%.2f%s\n",
                  static_cast<unsigned long>(canRxFrames), static_cast<unsigned long>(canTxErrors),
                  sourceVoltage, BoardConfig::kBatteryAdc < 0 ? " (nominal)" : "");
}

void setup()
{
    Serial.begin(115200);
    const uint32_t startedAt = millis();
    while (!Serial && millis() - startedAt < 3000) delay(10);
    Serial.println("ESP32-S3 wheel-legged controller; boot DISARMED");
    pinMode(BoardConfig::kButtonCenter, INPUT_PULLUP);
    pinMode(BoardConfig::kButtonStop, INPUT_PULLUP);
    commandQueue = xQueueCreate(8, sizeof(UserCommand));
    motionQueue = xQueueCreate(1, sizeof(MotionCommand));
    if (commandQueue == nullptr || motionQueue == nullptr) {
        Serial.println("Command queue allocation failed");
        while (true) delay(1000);
    }
    for (size_t i = 0; i < 6; i++) {
        if (BoardConfig::kMotorIds[i] < 1 || BoardConfig::kMotorIds[i] > 8) {
            Serial.println("Invalid motor ID; expected 1..8");
            while (true) delay(1000);
        }
        for (size_t j = 0; j < i; j++) {
            if (BoardConfig::kMotorIds[i] == BoardConfig::kMotorIds[j]) {
                Serial.println("Duplicate configured motor IDs");
                while (true) delay(1000);
            }
        }
    }
    Motor_InitAll();
    ADC_Init();
    CAN_Init();
    IMU_Init();
    Ctrl_Init();
    BT_Init();
    Serial_Init();
    xTaskCreatePinnedToCore(Motor_SendTask, "Motor_SendTask", 4096, nullptr, 5, nullptr, 1);
    Serial.println("Commands (newline): status, imu-config, arm, stop, stand, heading");
}

void loop()
{
    if (digitalRead(BoardConfig::kButtonStop) == LOW) Control_Stop();
    static bool centerWasPressed = false;
    const bool centerPressed = digitalRead(BoardConfig::kButtonCenter) == LOW;
    if (centerPressed && !centerWasPressed) Handle_Command(UserCommand::Heading);
    centerWasPressed = centerPressed;
    delay(20);
}
