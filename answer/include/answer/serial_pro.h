//
// Created by roc on 25-2-25.
//

#ifndef SERIAL_PRO_HPP
#define SERIAL_PRO_HPP

#include <string>
#include <functional>
#include <vector>

namespace sp {

    template<typename HEAD, typename TAIL>
    class serialPro {
    public:
        serialPro() = default;
        serialPro(std::string port, int baudrate);
        virtual ~serialPro();

        // 注册校验器
        void registerChecker(std::function<int(const HEAD&)> headChecker);
        void registerChecker(std::function<int(const TAIL&, const uint8_t*, const int&)> tailChecker);

        // 设置获取ID和长度的函数
        void setGetId(std::function<int(const HEAD&)> getId);
        void setGetLength(std::function<int(const HEAD&)> getLength);

        // 打开和关闭串口
        bool open();
        void close();

        // 发送和接收数据
        bool write(const std::vector<uint8_t>& data);
        std::vector<uint8_t> read(size_t length);

    protected:
        int fd_;
        std::string port_;
        int baudrate_;

        std::function<int(const HEAD&)> headChecker_;
        std::function<int(const TAIL&, const uint8_t*, const int&)> tailChecker_;
        std::function<int(const HEAD&)> getId_;
        std::function<int(const HEAD&)> getLength_;
    };

} // namespace sp

#endif