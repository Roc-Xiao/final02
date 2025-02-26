//
// Created by roc on 25-2-25.
//

#ifndef MY_SERIAL_HPP
#define MY_SERIAL_HPP

#include "serial_pro.hpp"
#include <cstdint>

namespace my_serial {

    struct Head {
        uint64_t SOF = 0xAA;    // 头校验
        uint64_t length = 0;     // 长度
        uint64_t cmd_id = 0x00;  // 命令字
    };

    struct password_send_t {
        uint64_t password1;    // 密码片段
        uint64_t password2;
    };

    struct password_receive_t {
        uint64_t password;    // 密码
    };

    struct Tail {
        uint64_t crc16 = 0xBB;    // 尾校验
    };

    class MySerial : public sp::serialPro<Head, Tail> {
    public:
        MySerial() = default;
        MySerial(std::string s, int band);

        // 发送密码片段
        bool sendPasswordFragments(uint64_t fragment1, uint64_t fragment2);
        // 接收完整密码
        int64_t receivePassword();
    };

} // namespace my_serial

#endif