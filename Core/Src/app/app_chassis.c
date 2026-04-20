#include "app/app_chassis.h"
#include "bsp/bsp_rc.h"
#include "can.h"
#include "tim.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1; // Usually M2006 on CAN1 or CAN2

motor_measure_t m2006_motors[4];
pid_type_def m2006_speed_pids[4];
static uint32_t motor_last_update_ms[4] = {0};
static uint32_t can_tx_fail_count = 0U;
static volatile uint8_t can_recover_request = 0U;

#define CHASSIS_MOTOR_OFFLINE_MS 50U
#define CHASSIS_RC_OFFLINE_MS    100U
#define CHASSIS_WHEEL_SPEED_MAX  7000

// PID Calculation
static float pid_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL) return 0.0f;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];

    // Limit integral
    if (pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
    if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    // Limit total output
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    return pid->out;
}

// Init PIDs for M2006
void Motor_PID_Init(void)
{
    for (int i = 0; i < 4; i++)
    {
        m2006_speed_pids[i].Kp = 15.0f;     // Rough initial tuning for M2006 Speed
        m2006_speed_pids[i].Ki = 0.1f;
        m2006_speed_pids[i].Kd = 0.0f;
        m2006_speed_pids[i].max_out = 5000.f;  // C610 takes max current +/- ~10000
        m2006_speed_pids[i].max_iout = 3000.f; // Integral Limit
    }
}

// Mecanum Wheel Kinematics Inverse Matrix (Classic Layout)
// Coordinates: 
// vx is forward, vy is strafe left/right, vw is rotation counter-clockwise
void Chassis_Kinematics(int16_t vx, int16_t vy, int16_t vw, int16_t wheel_speed[4])
{
    // Wheel order depends on your physical installation (typically: 1: FL, 2: FR, 3: BR, 4: BL)
    // Please adapt the signs depending on wheel orientation
    wheel_speed[0] =  vx - vy - vw; // Front Left````
    wheel_speed[1] = -vx - vy - vw; // Front Right (Inverted typically, but depends on mounting)
    wheel_speed[2] = -vx + vy - vw; // Back Right  (Inverted typically)
    wheel_speed[3] =  vx + vy - vw; // Back Left
}

// Send Current to C610 (CAN ID 0x200 contains ID 1~4)
void CAN_Send_Drive_Currents(int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t send_mail_box;
    uint8_t tx_data[8];

    tx_header.StdId = 0x200; // Identifier for C610 IDs 1-4
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = (current1 >> 8) & 0xff; tx_data[1] = current1 & 0xff;
    tx_data[2] = (current2 >> 8) & 0xff; tx_data[3] = current2 & 0xff;
    tx_data[4] = (current3 >> 8) & 0xff; tx_data[5] = current3 & 0xff;
    tx_data[6] = (current4 >> 8) & 0xff; tx_data[7] = current4 & 0xff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        can_tx_fail_count++;
        return;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box) != HAL_OK)
    {
        can_tx_fail_count++;
    }
}

// Decoder callback for Motor Feedback from CAN RX
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
        {
            uint8_t idx = rx_header.StdId - 0x201;
            m2006_motors[idx].rotor_angle   = ((rx_data[0] << 8) | rx_data[1]);
            m2006_motors[idx].rotor_speed   = ((rx_data[2] << 8) | rx_data[3]);
            m2006_motors[idx].torque_current = ((rx_data[4] << 8) | rx_data[5]);
            m2006_motors[idx].temp          = rx_data[6];
            motor_last_update_ms[idx] = HAL_GetTick();
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return;
    }

    if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0U)
    {
        can_recover_request = 1U;
    }
}

// RTOS Task
void Chassis_Task(void * argument)
{
    int16_t target_wheel_speeds[4] = {0};
    int16_t motor_currents[4]      = {0};
    uint16_t servo_pulse           = 700; // Initialize at your mid/safe position
    RC_ctrl_t rc_snapshot;
    
    Motor_PID_Init();

    // Setup CAN filter (accept all for demo)
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF);

    // Init Servo PWM (TIM1 CH1)
    // TIM1 clock is 168MHz. Prescaler = 168-1 = 167 -> 1MHz counter clock.
    // Period = 20000-1 = 19999 -> 50Hz (20ms) PWM signal.
    __HAL_TIM_SET_PRESCALER(&htim1, 167);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 19999);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 700); // 1.5ms pulse -> Middle position
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    while(1)
    {
        __disable_irq();
        rc_snapshot = rc_ctrl;
        __enable_irq();

        // 1. Get Joystick targets 
        // ch[1] > 0 对应向前推 -> 前进 (vx为正)
        int16_t vx = rc_snapshot.rc.ch[1]; 
        
        // ch[0] < 0 对应向左推 -> 左平移 (vy为正) 
        // 因为大疆遥控器向左推的时候通道读数是小于0的，而我们的运动学代码中 vy > 0 是向左，所以加上负号
        int16_t vy = -rc_snapshot.rc.ch[0]; 
        
        // ch[2] < 0 对应向左推 -> 原地左旋 (vw为正)
        // 同样左推数值为负，而运动学中 vw > 0 是左旋(逆时针)，所以加上负号
        int16_t vw = -rc_snapshot.rc.ch[2]; 

        // 2. Add Deadband to stick
        if (vx < 30 && vx > -30) vx = 0;
        if (vy < 30 && vy > -30) vy = 0;
        if (vw < 30 && vw > -30) vw = 0;

        // Multiply speed by simple coefficient. M2006 is 36:1, max rotor speed is ~10000 RPM 
        // -> Max wheel speed is around 250 RPM. DBUS range is +-660. 
        // Scale it safely. 
        vx *= 7; 
        vy *= 7;
        vw *= 7;

        if (vx > CHASSIS_WHEEL_SPEED_MAX) vx = CHASSIS_WHEEL_SPEED_MAX;
        if (vx < -CHASSIS_WHEEL_SPEED_MAX) vx = -CHASSIS_WHEEL_SPEED_MAX;
        if (vy > CHASSIS_WHEEL_SPEED_MAX) vy = CHASSIS_WHEEL_SPEED_MAX;
        if (vy < -CHASSIS_WHEEL_SPEED_MAX) vy = -CHASSIS_WHEEL_SPEED_MAX;
        if (vw > CHASSIS_WHEEL_SPEED_MAX) vw = CHASSIS_WHEEL_SPEED_MAX;
        if (vw < -CHASSIS_WHEEL_SPEED_MAX) vw = -CHASSIS_WHEEL_SPEED_MAX;

        {
            uint32_t now_ms = HAL_GetTick();
            uint8_t rc_online = ((now_ms - rc_last_update_ms) <= CHASSIS_RC_OFFLINE_MS);
            uint8_t motor_online = 1U;

            for (int i = 0; i < 4; i++)
            {
                if ((now_ms - motor_last_update_ms[i]) > CHASSIS_MOTOR_OFFLINE_MS)
                {
                    motor_online = 0U;
                    break;
                }
            }

            if ((rc_online == 0U) || (motor_online == 0U))
            {
                vx = 0;
                vy = 0;
                vw = 0;
            }
        }

        // Remote Switch Check (Safety disable) (rc_ctrl.rc.s[0] values: 1(UP), 3(MID), 2(DOWN))
        if(rc_snapshot.rc.s[0] == 2 ) { 
            // Right Stick is pointing DOWN. Stop the chassis motors for safety.
            vx = 0; vy = 0; vw = 0;
        }

        // Servo Gripper logic on Left Switch (s[1])
        // Increment/Decrement the pulse while the switch is held
        if (rc_snapshot.rc.s[1] == 1) {
            servo_pulse++; 
        } else if (rc_snapshot.rc.s[1] == 2) {
            servo_pulse--; 
        }
        
        // Add bounds for safety (using your tested limits)
        if (servo_pulse > 740) {
            servo_pulse = 740;
        } else if (servo_pulse < 550) {
            servo_pulse = 550;
        }

        // Set the PWM constantly so it holds its position in the middle (3)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo_pulse);

        // 3. Mecanum Kinematics calculation
        Chassis_Kinematics(vx, vy, vw, target_wheel_speeds);

        // 4. PID calculation for each motor
        for(int i = 0; i < 4; i++)
        {
            motor_currents[i] = pid_calc(&m2006_speed_pids[i], m2006_motors[i].rotor_speed, target_wheel_speeds[i]);
        }

        // 5. Send CAN Frame
        CAN_Send_Drive_Currents(motor_currents[0], motor_currents[1], motor_currents[2], motor_currents[3]);

        if ((can_recover_request != 0U) || (can_tx_fail_count > 100U))
        {
            HAL_CAN_Stop(&hcan1);
            HAL_CAN_Start(&hcan1);
            HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF);
            can_tx_fail_count = 0U;
            can_recover_request = 0U;
        }

        osDelay(2); // Runs at 500Hz
    }
}