#pragma once

#include <string>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

class RigctlPTT {
public:
    RigctlPTT(const std::string& host = "localhost", int port = 4532)
        : host_(host), port_(port) {}
    
    ~RigctlPTT() {
        disconnect();
    }
    
    bool connect() {
        if (connected_) return true;
        
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) {
            std::cerr << "rigctl: Failed to create socket" << std::endl;
            return false;
        }
        
        struct hostent* server = gethostbyname(host_.c_str());
        if (!server) {
            std::cerr << "rigctl PTT: Can't connect to host " << host_ << std::endl;
            close(sock_);
            sock_ = -1;
            return false;
        }
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        memcpy(&addr.sin_addr.s_addr, server->h_addr, server->h_length);
        addr.sin_port = htons(port_);
        
        if (::connect(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "rigctl: Can't connect to " << host_ << ":" << port_ << std::endl;
            close(sock_);
            sock_ = -1;
            return false;
        }
        
        connected_ = true;
        std::cerr << "rigctl: Connected to " << host_ << ":" << port_ << std::endl;
        return true;
    }


    
    void disconnect() {
        if (sock_ >= 0) {
            if (ptt_on_) {
                set_ptt(false);
            }
            close(sock_);
            sock_ = -1;
        }
        connected_ = false;
    }


    
    bool set_ptt(bool on) {
        if (!connected_) {
            if (!connect()) return false;
        }
        
        // T 1 (PTT on) or T 0 (PTT off)
        std::string cmd = on ? "T 1\n" : "T 0\n";
        
        if (send(sock_, cmd.c_str(), cmd.length(), 0) < 0) {
            std::cerr << "rigctl: Failed to send PTT command" << std::endl;
            disconnect();
            return false;
        }
        
        // read response
        char response[256];
        int n = recv(sock_, response, sizeof(response) - 1, 0);
        if (n > 0) {
            response[n] = '\0';
            // rigctld returns RPRT 0 on success
            if (strstr(response, "RPRT 0") || n == 0) {
                ptt_on_ = on;
                std::cerr << "rigctl: PTT " << (on ? "ON" : "OFF") << std::endl;
                return true;
            } else {
                std::cerr << "rigctl: PTT command failed: " << response << std::endl;
                return false;
            }
        }
        // temp fallback
        ptt_on_ = on;
        return true;
    }

    
    bool ptt_on() const { return ptt_on_; }
    bool is_connected() const { return connected_; }


private:
    std::string host_;
    int port_;
    int sock_ = -1;
    bool connected_ = false;
    bool ptt_on_ = false;
};



class DummyPTT {
public:
    bool connect() { 
        std::cerr << "PTT: Using dummy PTT (no rigctld)" << std::endl;
        return true; 
    }
    void disconnect() {}
    bool set_ptt(bool on) { 
        ptt_on_ = on;
        std::cerr << "PTT: " << (on ? "ON" : "OFF") << " (dummy)" << std::endl;
        return true; 
    }
    bool ptt_on() const { return ptt_on_; }
    bool is_connected() const { return true; }
    
private:
    bool ptt_on_ = false;
};
