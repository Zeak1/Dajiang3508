/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Motor_Num 4
#define Motor_Radius 0.075f
#define Chassis_Dis 0.45f
#define Resolution 4096         // 编码器分辨率（脉冲/转）
#define M_PI 3.1415f
#define Gear_Ratio 19.19f        // 减速比

typedef struct {
    float x;
    float y;
}Vector2D;

typedef struct {
    float vx;
    float vy;
    float w;
}Velocity;

typedef struct PID{
	float kp, ki, kd;
	float thisError; float lastError; float llastError;
	float set, get, limitH,limitL,out;
}PID;

//单电机结构体
typedef struct Motor{
	int16_t angle;
	int16_t speed;
	int16_t current;
	int8_t temp;
	PID pid;
	uint8_t  enabledataH;
	uint8_t  enabledataL;
	uint32_t rxid;
}DjiMotor;

typedef struct POS{
	PID nowpid;
	uint32_t wantpos;
}pos;

typedef struct Position
{
	float rx;
	float ry;
}WheelPosition;

typedef struct coefficient
{
	float Ncos;
	float Nsin;
}N_C;

DjiMotor Motors[Motor_Num];

WheelPosition wheelPositions[Motor_Num] = {
    {0.325f, 0.325f},
    {-0.325f, 0.325f}, 
    {-0.325f, -0.325f}, 
    {0.325f, -0.325f} 
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

N_C N_coefficient[Motor_Num] = {
	{0.7072,-0.7072},
	{-0.7072,-0.7072},
	{-0.7072,0.7072},
	{0.7072,0.7072}
};
uint8_t djicmd[8]={0,0,0,0,0,0,0,0};//大疆电机控制命令

pos zpos;//爪子抬升
pos expect_speed;	//预期速度
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void mycan1Init(void);
void mycan1TX(uint32_t id ,uint8_t mytxbuffer[]);
void GetCAN1HeadAndBuffer(void*myhead,void*mybuffer);


void pidInit(PID*ptr,float kp,float ki,float kd,float limitH,float limitL);
void pidcalc(PID* ptr,float get,float set);


void djiInit(DjiMotor*ptr,float kp,float ki,float kd,float limitH,float limitL,uint32_t rxid);
void djienable(DjiMotor*ptr,int16_t speed);

void enable(void);

float dotProduct(Vector2D a, Vector2D b);
float magnitude(Vector2D v);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 计算两个二维向量的点积
float dotProduct(Vector2D a, Vector2D b) {
    return a.x * b.x + a.y * b.y;
}

// 计算二维向量的模长
float magnitude(Vector2D v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

void pidInit(PID*ptr,float kp,float ki,float kd,float limitH,float limitL){
	ptr->get=0;ptr->set=0;ptr->out=0;
	ptr->thisError=0;ptr->lastError=0;ptr->llastError=0;
	ptr->kp=kp;ptr->ki=ki;ptr->kd=kd;
	ptr->limitH=limitH;ptr->limitL=limitL;
}
void pidcalc(PID* ptr,float get,float set){
	ptr->get=get;
	ptr->set=set;
	ptr->thisError=ptr->set-ptr->get;
	ptr->out=ptr->kp*(ptr->thisError-ptr->lastError) + ptr->ki*ptr->thisError + ptr->kd*(ptr->thisError-2*ptr->lastError+ptr->llastError);
	if(ptr->out > ptr->limitH){
		ptr->out=ptr->limitH;
	}else if(ptr->out < ptr->limitL){
		ptr->out=ptr->limitL;
	}else{
		ptr->out=ptr->out;
	}
	ptr->llastError=ptr->lastError;
	ptr->lastError=ptr->thisError;
}

void djiInit(DjiMotor*ptr,float kp,float ki,float kd,float limitH,float limitL,uint32_t rxid){
	ptr->angle=0;
	ptr->speed=0;
	ptr->current=0;
	ptr->temp=0;
	pidInit(&ptr->pid,kp,ki,kd,limitH,limitL);
	ptr->enabledataH=0;
	ptr->enabledataL=0;
	ptr->rxid=rxid;
}
void djienable(DjiMotor*ptr,int16_t speed){
	pidcalc(&ptr->pid,ptr->speed,speed);
	ptr->enabledataH=(uint8_t)((int16_t)(ptr->pid.out)>>8);
	ptr->enabledataL=(uint8_t)((int16_t)(ptr->pid.out));
	djicmd[2*(ptr->rxid-0x201)]=ptr->enabledataH;
	djicmd[2*(ptr->rxid-0x201)+1]=ptr->enabledataL;
	mycan1TX(0x200,djicmd);
}

/*速度解算*/
// 目标速度Vo与滚子无滚动方向的角度θ
// 1-135 2-225 3-315 4-45
// 计算电机速度
void calculateMotorSpeeds(Velocity vel, float motorSpeeds[Motor_Num]) {
    for (int i = 0; i < Motor_Num; i++) {
        float rx = wheelPositions[i].rx;
        float ry = wheelPositions[i].ry;

        // 计算平动速度 V1
        Vector2D V1 = {vel.vx, vel.vy};

        // 计算转动线速度 V2
        Vector2D V2 = {-ry * vel.w, rx * vel.w};

        // 计算投影目标速度 V0
        Vector2D V0 = {V1.x + V2.x, V1.y + V2.y};

        // 计算滚子无滚动方向的单位向量 n
        Vector2D n = {N_coefficient[i].Ncos, N_coefficient[i].Nsin};

        // 计算平行速度 V_parallel
        float V_parallel_magnitude = dotProduct(V0, n);

        // 计算电机线速度 V_Final
        float V_Final = V_parallel_magnitude;

        // 计算电机旋转速度 V_Motor
        float V_Motor = (V_Final - vel.w*Chassis_Dis) / Motor_Radius;
				//转换为脉冲值
				float omega_motor = V_Motor * Gear_Ratio;

				float motorPulses = omega_motor * Resolution / (2 * M_PI);
        // 存储结果
        motorSpeeds[i] = motorPulses / 60;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	

	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */




  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	mycan1Init();  //CAN初始化	
	for(int i = 0 ; i < Motor_Num ; i++)
	{
		uint32_t rxid = 0x201 + i;
		djiInit(&Motors[i],25.0,10.0,0.0,8000.0,-8000.0,rxid);
		
	}
	pidInit(&zpos.nowpid,25.0,10.0,0.0,8000.0,-8000.0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */	
    static Velocity desiredVel = {0.6f, 0.6f, 0.0f};
    float motorSpeeds[Motor_Num];
    // 计算电机速度
    calculateMotorSpeeds(desiredVel, motorSpeeds);
    // 发送CAN消息到对应的电机
    for (int i = 0; i < Motor_Num; i++) 
		{
      djienable(&Motors[i], (int16_t)motorSpeeds[i]);
    }
    // 添加一个小延时以避免CPU占用过高
    HAL_Delay(2);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void mycan1Init(){
	//can1的滤波begin************************************************************
	CAN_FilterTypeDef sFilterConfig = {0};
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_StatusTypeDef ret = HAL_CAN_Start(&hcan1);
	//can1滤波end************************************************************************
	//启用can1
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL);
}



void mycan1TX(uint32_t id ,uint8_t mytxbuffer[]){  //can1发送
  CAN_TxHeaderTypeDef TX_Header ={0};
	TX_Header.StdId = id;
	TX_Header.ExtId = 0x000;
	TX_Header.IDE = CAN_ID_STD;
	TX_Header.RTR = CAN_RTR_DATA;
	TX_Header.DLC = 8;
	uint32_t mailbox = 0;
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {}
	HAL_CAN_AddTxMessage(&hcan1, &TX_Header, mytxbuffer, &mailbox);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){//can接收回调函数  FIFO0未满
	CAN_RxHeaderTypeDef heard ={0};
	CAN_RxHeaderTypeDef*ptr_heard=&heard;
	CAN_RxHeaderTypeDef heard01 ={0};
	CAN_RxHeaderTypeDef*ptr_heard01=&heard01;
	uint8_t buffer [8]={0};
	uint32_t fifoFillLevel = HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0);

	for(uint32_t i =0;i<fifoFillLevel;i++){
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&heard,buffer);
		//取出heard和buffer
		GetCAN1HeadAndBuffer(ptr_heard,buffer);
	}
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){//can接收回调函数  FIFO0已满
	CAN_RxHeaderTypeDef heard ={0};
	CAN_RxHeaderTypeDef*ptr_heard=&heard;
	CAN_RxHeaderTypeDef heard01 ={0};
	CAN_RxHeaderTypeDef*ptr_heard01=&heard01;
	uint8_t buffer [8]={0};
	uint32_t fifoFillLevel = HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0);
	for(uint32_t i =0;i<fifoFillLevel;i++){
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&heard,buffer);
		//取出heard和buffer
		GetCAN1HeadAndBuffer(ptr_heard,buffer);
	}
}



	void GetCAN1HeadAndBuffer(void*myhead,void*mybuffer){
		CAN_RxHeaderTypeDef*ptr_heard=(CAN_RxHeaderTypeDef*)myhead;
		uint8_t*buffer=(uint8_t*)mybuffer;

		if(ptr_heard->StdId==0x201){//左后轮数据拼接
			Motors[0].angle=(int16_t)(buffer[0]<<8|buffer[1]);
			Motors[0].speed=(int16_t)(buffer[2]<<8|buffer[3]);
			Motors[0].current=(int16_t)(buffer[4]<<8|buffer[5]);
			Motors[0].temp=(int8_t)buffer[6];
		}
		else if(ptr_heard->StdId==0x202){//右后轮数据拼接
			Motors[1].angle=(int16_t)(buffer[0]<<8|buffer[1]);
			Motors[1].speed=(int16_t)(buffer[2]<<8|buffer[3]);
			Motors[1].current=(int16_t)(buffer[4]<<8|buffer[5]);
			Motors[1].temp=(int8_t)buffer[6];
		}
		else if(ptr_heard->StdId==0x203){//右前数据拼接
			Motors[2].angle=(int16_t)(buffer[0]<<8|buffer[1]);
			Motors[2].speed=(int16_t)(buffer[2]<<8|buffer[3]);
			Motors[2].current=(int16_t)(buffer[4]<<8|buffer[5]);
			Motors[2].temp=(int8_t)buffer[6];
		}
		else if(ptr_heard->StdId==0x204){//右后轮数据拼接
			Motors[3].angle=(int16_t)(buffer[0]<<8|buffer[1]);
			Motors[3].speed=(int16_t)(buffer[2]<<8|buffer[3]);
			Motors[3].current=(int16_t)(buffer[4]<<8|buffer[5]);
			Motors[3].temp=(int8_t)buffer[6];
		}
	}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
