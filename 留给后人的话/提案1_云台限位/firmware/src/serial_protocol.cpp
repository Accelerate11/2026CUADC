#include "serial_protocol.h"

#include <cstdio>
#include <cstring>

namespace { SerialProtocol* g_protocol = nullptr; }

SerialProtocol::SerialProtocol(UART_HandleTypeDef& uart) : uart_(uart) {}

void SerialProtocol::begin() {
    g_protocol = this;
    (void)HAL_UART_Receive_IT(&uart_, &rx_byte_, 1U);
}

void SerialProtocol::handleRxComplete(UART_HandleTypeDef* uart) {
    if (uart == &uart_) {
        pushByte(rx_byte_);
        (void)HAL_UART_Receive_IT(&uart_, &rx_byte_, 1U);
    }
}

void SerialProtocol::handleTxComplete(UART_HandleTypeDef* uart) {
    if (uart != &uart_ || !tx_busy_) return;
    tx_tail_ = static_cast<uint16_t>((tx_tail_ + tx_active_length_) % kTxQueueSize);
    tx_active_length_ = 0U;
    tx_busy_ = false;
    kickTx();
}

void SerialProtocol::pushByte(uint8_t byte) {
    if (byte == '\r') return;
    if (byte == '\n') {
        if (line_length_ > 0U && !line_ready_) {
            std::memcpy(complete_line_, line_, line_length_);
            complete_line_[line_length_] = '\0';
            line_ready_ = true;
        }
        line_length_ = 0U;
        return;
    }
    if (line_length_ < kLineBufferSize - 1U) {
        line_[line_length_++] = static_cast<char>(byte);
    } else {
        line_length_ = 0U;
    }
}

bool SerialProtocol::parseLine(char* line, Command& command) {
    command = {};
    if (std::strcmp(line, "ARM") == 0) command.type = CommandType::Arm;
    else if (std::strcmp(line, "DISARM") == 0) command.type = CommandType::Disarm;
    else if (std::strcmp(line, "ZERO") == 0) command.type = CommandType::Zero;
    else if (std::strcmp(line, "STATUS?") == 0) command.type = CommandType::Status;
    else if (std::sscanf(line, "SET,%f,%f", &command.a, &command.b) == 2)
        command.type = CommandType::SetTarget;
    else if (std::sscanf(line, "GAIN,R,%f,%f,%f,%f",
                         &command.a, &command.b, &command.c, &command.d) == 4)
        command.type = CommandType::SetRollGains;
    else if (std::sscanf(line, "GAIN,P,%f,%f,%f,%f",
                         &command.a, &command.b, &command.c, &command.d) == 4)
        command.type = CommandType::SetPitchGains;
    else command.type = CommandType::Invalid;
    return true;
}

bool SerialProtocol::poll(Command& command) {
    if (!line_ready_) return false;
    const bool parsed = parseLine(complete_line_, command);
    line_ready_ = false;
    return parsed;
}

void SerialProtocol::sendLine(const char* text) {
    char framed[kTxBufferSize]{};
    const int requested = std::snprintf(framed, sizeof(framed), "%s\r\n", text);
    if (requested <= 0 || requested >= static_cast<int>(sizeof(framed))) return;
    const uint16_t length = static_cast<uint16_t>(requested);

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint16_t used = static_cast<uint16_t>((tx_head_ + kTxQueueSize - tx_tail_) % kTxQueueSize);
    const uint16_t available = static_cast<uint16_t>(kTxQueueSize - used - 1U);
    if (length <= available) {
        for (uint16_t i = 0U; i < length; ++i) {
            tx_queue_[tx_head_] = static_cast<uint8_t>(framed[i]);
            tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxQueueSize);
        }
        kickTx();
    }
    if (primask == 0U) __enable_irq();
}

void SerialProtocol::kickTx() {
    if (tx_busy_ || tx_head_ == tx_tail_) return;
    tx_active_length_ = tx_head_ > tx_tail_
        ? static_cast<uint16_t>(tx_head_ - tx_tail_)
        : static_cast<uint16_t>(kTxQueueSize - tx_tail_);
    tx_busy_ = true;
    if (HAL_UART_Transmit_IT(&uart_, &tx_queue_[tx_tail_], tx_active_length_) != HAL_OK) {
        tx_active_length_ = 0U;
        tx_busy_ = false;
    }
}

void SerialProtocol::sendAck(const char* command) {
    char buffer[64]{};
    (void)std::snprintf(buffer, sizeof(buffer), "ACK,%s", command);
    sendLine(buffer);
}

void SerialProtocol::sendError(const char* reason) {
    char buffer[96]{};
    (void)std::snprintf(buffer, sizeof(buffer), "ERR,%s", reason);
    sendLine(buffer);
}

const char* SerialProtocol::stateName(SystemState state) {
    switch (state) {
    case SystemState::Boot: return "BOOT";
    case SystemState::Calibrating: return "CALIBRATING";
    case SystemState::Ready: return "READY";
    case SystemState::Armed: return "ARMED";
    case SystemState::Fault: return "FAULT";
    default: return "UNKNOWN";
    }
}

void SerialProtocol::sendTelemetry(SystemState system_state, const AttitudeState& attitude,
                                   const ServoAngles& servo, uint32_t imu_failures) {
    char buffer[kTxBufferSize]{};
    (void)std::snprintf(buffer, sizeof(buffer),
        "TEL,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%lu",
        stateName(system_state), static_cast<double>(attitude.roll_deg),
        static_cast<double>(attitude.pitch_deg), static_cast<double>(attitude.roll_rate_dps),
        static_cast<double>(attitude.pitch_rate_dps), static_cast<double>(attitude.accel_norm_g),
        static_cast<double>(servo.roll_deg), static_cast<double>(servo.pitch_deg),
        static_cast<unsigned long>(imu_failures));
    sendLine(buffer);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart) {
    if (g_protocol != nullptr) g_protocol->handleRxComplete(uart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart) {
    if (g_protocol != nullptr) g_protocol->handleTxComplete(uart);
}
