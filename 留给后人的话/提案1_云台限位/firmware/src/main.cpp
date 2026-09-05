#include "app_config.h"
#include "attitude_filter.h"
#include "gimbal_controller.h"
#include "icm42688.h"
#include "serial_protocol.h"
#include "servo_pwm.h"

#include "stm32g4xx_hal.h"

#include <cmath>

namespace {
SPI_HandleTypeDef g_spi1{};
UART_HandleTypeDef g_uart1{};
TIM_HandleTypeDef g_tim2{};
TIM_HandleTypeDef g_tim6{};
volatile uint32_t g_control_ticks = 0U;

void failFast() {
    __disable_irq();
    while (true) {}
}

void systemClockInit() {
    RCC_OscInitTypeDef osc{};
    RCC_ClkInitTypeDef clk{};

    (void)HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = RCC_PLLM_DIV4;
    osc.PLL.PLLN = 85U;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) failFast();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) failFast();
}

void gpioInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void spiInit() {
    __HAL_RCC_SPI1_CLK_ENABLE();
    g_spi1.Instance = SPI1;
    g_spi1.Init.Mode = SPI_MODE_MASTER;
    g_spi1.Init.Direction = SPI_DIRECTION_2LINES;
    g_spi1.Init.DataSize = SPI_DATASIZE_8BIT;
    g_spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_spi1.Init.NSS = SPI_NSS_SOFT;
    g_spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    g_spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spi1.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_spi1.Init.CRCPolynomial = 7U;
    g_spi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    g_spi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&g_spi1) != HAL_OK) failFast();
}

void uartInit() {
    __HAL_RCC_LPUART1_CLK_ENABLE();
    g_uart1.Instance = LPUART1;
    g_uart1.Init.BaudRate = config::kUartBaud;
    g_uart1.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart1.Init.StopBits = UART_STOPBITS_1;
    g_uart1.Init.Parity = UART_PARITY_NONE;
    g_uart1.Init.Mode = UART_MODE_TX_RX;
    g_uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart1.Init.OverSampling = UART_OVERSAMPLING_16;
    g_uart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    g_uart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&g_uart1) != HAL_OK) failFast();
    HAL_NVIC_SetPriority(LPUART1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}

void pwmTimerInit() {
    __HAL_RCC_TIM2_CLK_ENABLE();
    g_tim2.Instance = TIM2;
    g_tim2.Init.Prescaler = 169U; // 170 MHz / 170 = 1 MHz
    g_tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim2.Init.Period = config::kServoPeriodUs - 1U;
    g_tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&g_tim2) != HAL_OK) failFast();

    TIM_OC_InitTypeDef channel{};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 1500U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&g_tim2, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&g_tim2, &channel, TIM_CHANNEL_2) != HAL_OK) failFast();
}

void controlTimerInit() {
    __HAL_RCC_TIM6_CLK_ENABLE();
    g_tim6.Instance = TIM6;
    g_tim6.Init.Prescaler = 169U;
    g_tim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim6.Init.Period = 2000U - 1U; // 500 Hz
    g_tim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&g_tim6) != HAL_OK) failFast();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

bool stationaryForCalibration(const ImuSample& sample) {
    const float norm = std::sqrt(sample.accel_x_g * sample.accel_x_g +
                                 sample.accel_y_g * sample.accel_y_g +
                                 sample.accel_z_g * sample.accel_z_g);
    return std::fabs(norm - 1.0F) <= config::kCalibrationAccelToleranceG &&
           std::fabs(sample.gyro_x_dps) <= config::kCalibrationMaxGyroDps &&
           std::fabs(sample.gyro_y_dps) <= config::kCalibrationMaxGyroDps &&
           std::fabs(sample.gyro_z_dps) <= config::kCalibrationMaxGyroDps;
}

bool validGains(const Command& command) {
    return command.a > 0.0F && command.a <= 20.0F &&
           command.b > 0.0F && command.b <= 10.0F &&
           command.c >= 10.0F && command.c <= 300.0F &&
           command.d >= 10.0F && command.d <= 500.0F;
}
} // namespace

extern "C" void SysTick_Handler() { HAL_IncTick(); }
extern "C" void LPUART1_IRQHandler() { HAL_UART_IRQHandler(&g_uart1); }
extern "C" void TIM6_DAC_IRQHandler() { HAL_TIM_IRQHandler(&g_tim6); }
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer) {
    if (timer->Instance == TIM6) ++g_control_ticks;
}

int main() {
    HAL_Init();
    systemClockInit();
    gpioInit();
    spiInit();
    uartInit();
    pwmTimerInit();
    controlTimerInit();

    SerialProtocol serial(g_uart1);
    ServoPwm roll_servo(g_tim2, TIM_CHANNEL_1, config::kRollMinPulseUs,
                        config::kRollCenterPulseUs, config::kRollMaxPulseUs);
    ServoPwm pitch_servo(g_tim2, TIM_CHANNEL_2, config::kPitchMinPulseUs,
                         config::kPitchCenterPulseUs, config::kPitchMaxPulseUs);
    Icm42688 imu(g_spi1, GPIOB, GPIO_PIN_6);
    AttitudeFilter filter;
    GimbalController controller;

    serial.begin();
    (void)roll_servo.begin();
    (void)pitch_servo.begin();
    controller.reset();

    SystemState system_state = SystemState::Boot;
    if (!imu.begin()) {
        system_state = SystemState::Fault;
        serial.sendError("IMU_INIT");
    } else {
        system_state = SystemState::Calibrating;
        serial.sendLine("INFO,CALIBRATION_KEEP_STILL");
    }

    if (HAL_TIM_Base_Start_IT(&g_tim6) != HAL_OK) failFast();

    uint32_t processed_tick = 0U;
    uint32_t calibration_count = 0U;
    float gyro_sum_x = 0.0F;
    float gyro_sum_y = 0.0F;
    float gyro_sum_z = 0.0F;
    float gyro_bias_x = 0.0F;
    float gyro_bias_y = 0.0F;
    float gyro_bias_z = 0.0F;
    uint32_t consecutive_imu_failures = 0U;
    AttitudeState attitude{};
    ServoAngles servo{};

    while (true) {
        Command command{};
        if (serial.poll(command)) {
            switch (command.type) {
            case CommandType::Arm:
                if (system_state == SystemState::Ready) {
                    controller.reset(roll_servo.angleDeg(), pitch_servo.angleDeg());
                    system_state = SystemState::Armed;
                    serial.sendAck("ARM");
                } else serial.sendError("NOT_READY");
                break;
            case CommandType::Disarm:
                if (system_state == SystemState::Armed) system_state = SystemState::Ready;
                serial.sendAck("DISARM");
                break;
            case CommandType::SetTarget:
                controller.setTarget(command.a, command.b);
                serial.sendAck("SET");
                break;
            case CommandType::Zero:
                if (system_state == SystemState::Ready &&
                    std::fabs(attitude.roll_rate_dps) < 2.0F &&
                    std::fabs(attitude.pitch_rate_dps) < 2.0F &&
                    std::fabs(attitude.accel_norm_g - 1.0F) < 0.05F) {
                    filter.setCurrentAsLevel();
                    serial.sendAck("ZERO");
                } else serial.sendError("ZERO_REQUIRES_STILL_DISARMED");
                break;
            case CommandType::SetRollGains:
            case CommandType::SetPitchGains:
                if (system_state != SystemState::Ready) serial.sendError("GAIN_REQUIRES_READY");
                else if (!validGains(command)) serial.sendError("GAIN_RANGE");
                else {
                    AxisGains gains{command.a, command.b, command.c, command.d};
                    if (command.type == CommandType::SetRollGains) controller.setRollGains(gains);
                    else controller.setPitchGains(gains);
                    serial.sendAck("GAIN");
                }
                break;
            case CommandType::Status:
                serial.sendTelemetry(system_state, attitude, servo, consecutive_imu_failures);
                break;
            case CommandType::Invalid:
                serial.sendError("BAD_COMMAND");
                break;
            default:
                break;
            }
        }

        const uint32_t available_tick = g_control_ticks;
        if (processed_tick == available_tick) continue;
        if (available_tick - processed_tick > 1U) processed_tick = available_tick - 1U;
        ++processed_tick;

        ImuSample sample{};
        if (!imu.read(sample, processed_tick * 2000U)) {
            ++consecutive_imu_failures;
            if (consecutive_imu_failures >= config::kMaxConsecutiveImuFailures) {
                system_state = SystemState::Fault;
                roll_servo.hold();
                pitch_servo.hold();
                if (consecutive_imu_failures == config::kMaxConsecutiveImuFailures) {
                    serial.sendError("IMU_READ");
                }
            }
            if (processed_tick % (config::kControlRateHz / config::kTelemetryRateHz) == 0U) {
                serial.sendTelemetry(system_state, attitude, servo, consecutive_imu_failures);
            }
            continue;
        }
        consecutive_imu_failures = 0U;

        if (system_state == SystemState::Calibrating) {
            if (!stationaryForCalibration(sample)) {
                calibration_count = 0U;
                gyro_sum_x = gyro_sum_y = gyro_sum_z = 0.0F;
            } else {
                gyro_sum_x += sample.gyro_x_dps;
                gyro_sum_y += sample.gyro_y_dps;
                gyro_sum_z += sample.gyro_z_dps;
                ++calibration_count;
                if (calibration_count >= config::kGyroCalibrationSamples) {
                    const float count = static_cast<float>(calibration_count);
                    gyro_bias_x = gyro_sum_x / count;
                    gyro_bias_y = gyro_sum_y / count;
                    gyro_bias_z = gyro_sum_z / count;
                    filter = AttitudeFilter{};
                    system_state = SystemState::Ready;
                    serial.sendLine("INFO,READY");
                }
            }
        }

        sample.gyro_x_dps -= gyro_bias_x;
        sample.gyro_y_dps -= gyro_bias_y;
        sample.gyro_z_dps -= gyro_bias_z;
        attitude = filter.update(sample, 1.0F / static_cast<float>(config::kControlRateHz));

        if (system_state == SystemState::Armed) {
            servo = controller.update(attitude, 1.0F / static_cast<float>(config::kControlRateHz));
            if (processed_tick % (config::kControlRateHz / config::kServoRateHz) == 0U) {
                roll_servo.setAngle(servo.roll_deg);
                pitch_servo.setAngle(servo.pitch_deg);
            }
        } else {
            servo.roll_deg = roll_servo.angleDeg();
            servo.pitch_deg = pitch_servo.angleDeg();
        }

        if (processed_tick % (config::kControlRateHz / config::kTelemetryRateHz) == 0U) {
            serial.sendTelemetry(system_state, attitude, servo, consecutive_imu_failures);
        }
    }
}
