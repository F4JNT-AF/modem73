#pragma once

#include <cstdint>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <functional>
#include <string>
#include <iomanip>
#include <iostream>

// KISS protocol
namespace KISS {
    constexpr uint8_t FEND  = 0xC0;
    constexpr uint8_t FESC  = 0xDB;
    constexpr uint8_t TFEND = 0xDC;
    constexpr uint8_t TFESC = 0xDD;
    
    // KISS commands
    constexpr uint8_t CMD_DATA     = 0x00;
    constexpr uint8_t CMD_TXDELAY  = 0x01;
    constexpr uint8_t CMD_P        = 0x02;
    constexpr uint8_t CMD_SLOTTIME = 0x03;
    constexpr uint8_t CMD_TXTAIL   = 0x04;
    constexpr uint8_t CMD_FULLDUPLEX = 0x05;
    constexpr uint8_t CMD_SETHW    = 0x06;
    constexpr uint8_t CMD_RETURN   = 0xFF;
}


enum class PTTType {
    NONE = 0,
    RIGCTL = 1,
    VOX = 2,
    COM = 3
};

struct TNCConfig {
    // Network settings
    std::string bind_address = "0.0.0.0";
    int port = 8001;
    
    // Audio settings
    std::string audio_input_device = "default";
    std::string audio_output_device = "default";
    int sample_rate = 48000;
    
    // Modem settings
    int center_freq = 1500;
    std::string callsign = "N0CALL";
    std::string modulation = "QPSK";
    std::string code_rate = "1/2";
    bool short_frame = false;  
    
    // PTT settings
    PTTType ptt_type = PTTType::RIGCTL;  
    
    // Rigctl settings 
    std::string rigctl_host = "localhost";
    int rigctl_port = 4532;
    
    // VOX settings 
    int vox_tone_freq = 1200;    // Hz - tone frequency for VOX trigger
    int vox_lead_ms = 550;       // ms - tone before OFDM data
    int vox_tail_ms = 500;       // ms - tone after OFDM data
    
    // COM/Serial PTT settings 
    std::string com_port = "/dev/ttyUSB0"; 
    int com_ptt_line = 1;        // 0=DTR, 1=RTS, 2=BOTH
    bool com_invert_dtr = false;
    bool com_invert_rts = false;
    
    // PTT timing 
    int ptt_delay_ms = 50;       // Delay after PTT before TX
    int ptt_tail_ms = 50;        // Delay after TX before PTT release
    
    // Operational settings
    int tx_delay_ms = 500;       // TXDelay 
    bool full_duplex = false;
    int slot_time_ms = 500;      // CSMA slot time
    int p_persistence = 128;     // 0-255 (128 defualt 50%)
    
    // CSMA settings
    bool csma_enabled = true;
    float carrier_threshold_db = -30.0f;  // Carrier sense threshold
    int carrier_sense_ms = 100;           // How long to listen
    int max_backoff_slots = 10;           // Maximum backoff
    
    // Settings file path
    std::string config_file = "";
};


class KISSParser {
public:
    using FrameCallback = std::function<void(uint8_t port, uint8_t cmd, const std::vector<uint8_t>&)>;
    
    KISSParser(FrameCallback callback) : callback_(callback) {}
    
    void process(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            process_byte(data[i]);
        }
    }
    
    static std::vector<uint8_t> wrap(const std::vector<uint8_t>& data, uint8_t port = 0) {
        std::vector<uint8_t> frame;
        frame.push_back(KISS::FEND);
        frame.push_back((port << 4) | KISS::CMD_DATA);
        
        for (uint8_t byte : data) {
            if (byte == KISS::FEND) {
                frame.push_back(KISS::FESC);
                frame.push_back(KISS::TFEND);
            } else if (byte == KISS::FESC) {
                frame.push_back(KISS::FESC);
                frame.push_back(KISS::TFESC);
            } else {
                frame.push_back(byte);
            }
        }
        
        frame.push_back(KISS::FEND);
        return frame;
    }
    
private:
    void process_byte(uint8_t byte) {
        if (byte == KISS::FEND) {
            if (in_frame_ && buffer_.size() > 0) {
                // frame complete
                uint8_t cmd_byte = buffer_[0];
                uint8_t port = (cmd_byte >> 4) & 0x0F;
                uint8_t cmd = cmd_byte & 0x0F;
                std::vector<uint8_t> payload(buffer_.begin() + 1, buffer_.end());
                callback_(port, cmd, payload);
            }
            in_frame_ = true;
            buffer_.clear();
            escape_ = false;
        } else if (in_frame_) {
            if (escape_) {
                if (byte == KISS::TFEND) {
                    buffer_.push_back(KISS::FEND);
                } else if (byte == KISS::TFESC) {
                    buffer_.push_back(KISS::FESC);
                } else {
                    buffer_.push_back(byte);
                }
                escape_ = false;
            } else if (byte == KISS::FESC) {
                escape_ = true;
            } else {
                buffer_.push_back(byte);
            }
        }
    }
    
    FrameCallback callback_;
    std::vector<uint8_t> buffer_;
    bool in_frame_ = false;
    bool escape_ = false;
};


template<typename T>
class PacketQueue {
public:
    void push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(item));
    }
    
    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) queue_.pop();
    }
    
private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
};


inline void hex_dump(const char* prefix, const uint8_t* data, size_t len) {
    std::cerr << prefix << " (" << len << " bytes):" << std::endl;
    for (size_t i = 0; i < len; i += 16) {
        std::cerr << "  " << std::hex << std::setfill('0') << std::setw(4) << i << ": ";
        
        // Hex
        for (size_t j = 0; j < 16 && i + j < len; ++j) {
            std::cerr << std::setw(2) << (int)data[i + j] << " ";
        }
        for (size_t j = len - i; j < 16 && i + j >= len; ++j) {
            std::cerr << "   ";
        }
        
        // ASCII
        std::cerr << " |";
        for (size_t j = 0; j < 16 && i + j < len; ++j) {
            char c = data[i + j];
            std::cerr << (c >= 32 && c < 127 ? c : '.');
        }
        std::cerr << "|" << std::endl;
    }
    std::cerr << std::dec;
}

// Length-prefix framing 
// This handles OFDM frame padding where the payload is encoded within the 2 byte prefix
inline std::vector<uint8_t> frame_with_length(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> framed;
    uint16_t len = data.size();
    framed.push_back((len >> 8) & 0xFF);  // high byte
    framed.push_back(len & 0xFF);          // low byte
    framed.insert(framed.end(), data.begin(), data.end());
    return framed;
}

inline std::vector<uint8_t> unframe_length(const uint8_t* data, size_t total_len) {
    if (total_len < 2) return {};
    uint16_t payload_len = (data[0] << 8) | data[1];
    if (payload_len > total_len - 2) {
        std::cerr << "Warning: length prefix " << payload_len 
                  << " exceeds available data " << (total_len - 2) << std::endl;
        payload_len = total_len - 2;
    }
    return std::vector<uint8_t>(data + 2, data + 2 + payload_len);
}
