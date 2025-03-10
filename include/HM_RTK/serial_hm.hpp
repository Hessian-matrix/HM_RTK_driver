#ifndef SERIAL_HESSIAN_H
#define SERIAL_HESSIAN_H

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <exception>
#include <string>
#include <sstream>

namespace Hessian {

    /// SerialException 用于串口操作中抛出错误信息
    class SerialException : public std::exception {
    public:
        SerialException(const std::string &name, const char *description) {
            std::stringstream ss;
            ss << "SerialException " << name << " >>> " << description << " failed.";
            e_what_ = ss.str();
        }
        SerialException(const SerialException &other) : e_what_(other.e_what_) {}
        virtual ~SerialException() throw() {}
        virtual const char* what() const throw() {
            return e_what_.c_str();
        }
    private:
        SerialException& operator=(const SerialException&);
        std::string e_what_;
    };

    /// Serial 类用于管理串口设备的打开、读写及关闭操作
    class Serial {
    public:
        Serial() {}
        ~Serial();
        Serial(const std::string &port, uint32_t baudrate, uint64_t read_timeout = 1000 /* ms */, 
               uint64_t write_timeout = 1000 /* ms */, int nBits = 8, char nEvent = 'N', int nStop = 1);
        void reset(const std::string &port, uint32_t baudrate, uint64_t read_timeout = 1000 /* ms */, 
                   uint64_t write_timeout = 1000 /* ms */, int nBits = 8, char nEvent = 'N', int nStop = 1);
        int write(const std::string &data);
        std::string read(int length);
        std::string readline(int length);
        bool isOpen() const { return _fd > 0; }
        bool close() { if (_fd) uart_close(); return true; }
    
    private:
        Serial(const Serial&) {}
        Serial& operator=(const Serial&) { return *this; }
        void whenError(const std::string &errInfo);
        int uart_open(const char *pathname);
        int uart_set(int nSpeed, int nBits, char nEvent, int nStop);
        int uart_close();
        int uart_send(const char *send_buffer, int length);
        int uart_recv(char *recv_buffer, int length);
    
    private:
        int _fd = 0;
        std::string description = "";
        std::string _port = "/dev/ttyS4";
        uint32_t _baudrate = 460800;
        uint64_t _read_timeout = 1000;
        uint64_t _write_timeout = 1000;
        int _nBits = 8; 
        char _nEvent = 'N'; 
        int _nStop = 1;
    };

} // namespace Hessian

#endif