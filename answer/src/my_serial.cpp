//
// Created by roc on 25-2-25.
//
#include "answer/my_serial.hpp"

namespace my_serial {

    MySerial::MySerial(std::string s, int band)
        : sp::serialPro<Head, Tail>(s, band) {

        registerChecker([](const Head& head)-> int {
            return head.SOF != 0xAA;
        });

        registerChecker([](const Tail& tail, const uint8_t*, const int&)-> int {
            return tail.crc16 != 0xBB;
        });

        setGetId([](const Head& head)-> int {
            return head.cmd_id;
        });

        setGetLength([](const Head& head)-> int {
            return (int)head.length;
        });
    }

    bool MySerial::sendPasswordFragments(uint64_t fragment1, uint64_t fragment2) {
        Head head;
        head.length = sizeof(password_send_t);
        head.cmd_id = 0x00;

        password_send_t data{fragment1, fragment2};
        Tail tail;

        std::vector<uint8_t> buffer;
        buffer.resize(sizeof(Head) + sizeof(password_send_t) + sizeof(Tail));

        memcpy(buffer.data(), &head, sizeof(Head));
        memcpy(buffer.data() + sizeof(Head), &data, sizeof(password_send_t));
        memcpy(buffer.data() + sizeof(Head) + sizeof(password_send_t), &tail, sizeof(Tail));

        return write(buffer);
    }

    int64_t MySerial::receivePassword() {
        auto header = read(sizeof(Head));
        if (header.empty()) return -1;

        Head head;
        memcpy(&head, header.data(), sizeof(Head));

        if (head.SOF != 0xAA || head.cmd_id != 0x01) {
            return -1;
        }

        auto data = read(sizeof(password_receive_t));
        auto tail_data = read(sizeof(Tail));

        if (data.empty() || tail_data.empty()) {
            return -1;
        }

        Tail tail;
        memcpy(&tail, tail_data.data(), sizeof(Tail));
        if (tail.crc16 != 0xBB) {
            return -1;
        }

        password_receive_t pwd;
        memcpy(&pwd, data.data(), sizeof(password_receive_t));
        return pwd.password;
    }

} // namespace my_serial