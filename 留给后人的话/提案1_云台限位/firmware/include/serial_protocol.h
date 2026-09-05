#pragma once

#include "gimbal_controller.h"
#include "stm32g4xx_hal.h"
#include "types.h"

enum class CommandType : uint8_t {
    None,
    Arm,
    Disarm,
    SetTarget,
    Zero,
    Status,
    SetRollGains,
    SetPitchGains,
    Invalid,
};

struct Command {
    CommandType type{CommandType::None};
    float a{0.0F};
    float b{0.0F};
    float c{0.0F};
    float d{0.0F};
};

class SerialProtocol {
public:
    explicit SerialProtocol(UART_HandleTypeDef& uart);
    void begin();
    void handleRxComplete(UART_HandleTypeDef* uart);
    void handleTxComplete(UART_HandleTypeDef* uart);
    bool poll(Command& command);
    void sendLine(const char* text);
    void sendAck(const char* command);
    void sendError(const char* reason);
    void sendTelemetry(SystemState system_state, const AttitudeState& attitude,
                       const ServoAngles& servo, uint32_t imu_failures);

private:
    static constexpr uint16_t kLineBufferSize = 128U;
    static constexpr uint16_t kTxBufferSize = 256U;
    static constexpr uint16_t kTxQueueSize = 1024U;

    bool parseLine(char* line, Command& command);
    void pushByte(uint8_t byte);
    void kickTx();
    static const char* stateName(SystemState state);

    UART_HandleTypeDef& uart_;
    uint8_t rx_byte_{0U};
    volatile bool rx_ready_{false};
    char line_[kLineBufferSize]{};
    uint16_t line_length_{0U};
    char complete_line_[kLineBufferSize]{};
    volatile bool line_ready_{false};
    uint8_t tx_queue_[kTxQueueSize]{};
    volatile uint16_t tx_head_{0U};
    volatile uint16_t tx_tail_{0U};
    volatile uint16_t tx_active_length_{0U};
    volatile bool tx_busy_{false};
};
