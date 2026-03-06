
#include "HM_RTK/serial_hm.hpp"
#include <chrono>
#include <iostream>
#include <vector>

// #define THROW_EXCEPTION

namespace Hessian{
    static constexpr int kMillisecToMicrosec = 1000;

    int64_t getCurTime(){
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    Serial::Serial(const std::string &port, uint32_t baudrate, uint64_t read_timeout, uint64_t write_timeout,
                int nBits, char nEvent, int nStop){
        reset(port, baudrate, read_timeout, write_timeout, nBits, nEvent, nStop);
    }

    void Serial::reset(const std::string &port, uint32_t baudrate, uint64_t read_timeout, uint64_t write_timeout,
                int nBits, char nEvent, int nStop){
        if(_fd) uart_close();

        try {
            _fd = uart_open(port.c_str());
            if(_fd == -1){
                std::cerr << "[HM_RTK ERROR] Serial port open failed: " << port << std::endl;
                exit(EXIT_FAILURE);
            }
            if(uart_set(baudrate,nBits,nEvent,nStop) == -1)
            {
                std::cerr << "[HM_RTK ERROR] Serial port configuration failed!" << std::endl;
                exit(EXIT_FAILURE);
            }

            _port = port;
            _baudrate = baudrate;
            _read_timeout = read_timeout;
            _write_timeout = write_timeout;
            std::cout << "[HM_RTK] Write timeout set to: " << _write_timeout << "ms" << std::endl;
            _nBits = nBits;
            _nEvent = nEvent;
            _nStop = nStop;
            std::stringstream ss;
            ss << "{" << _fd << "}" << port << ":" << baudrate << "[" << nBits << "," << nEvent << "," << nStop << "]";
            description = ss.str();
            std::cout << "[HM_RTK] Serial port configured: " << description << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[HM_RTK ERROR] Serial reset failed: " << e.what() << std::endl;
            _fd = 0;  // 确保文件描述符重置
            throw;  // 重新抛出异常
        }
        
    }

    Serial::~Serial(){
        if(_fd) uart_close();
    }

    
    void Serial::whenError(const std::string& errInfo){
#ifdef THROW_EXCEPTION
        throw(SerialException(description, errInfo.c_str()));
#else
        std::cerr << "[HM_RTK ERROR] " << description << " : " << errInfo << std::endl;
        reset(_port, _baudrate, _read_timeout, _write_timeout, _nBits, _nEvent, _nStop);
#endif 
    }

    int Serial::write(const std::string& data){
        const char* ptr = data.c_str();
        int n_count_send = 0, n_all = data.size();
        int64_t t_start = getCurTime();
        while(n_count_send < n_all && getCurTime() - t_start < _write_timeout){
            int len = uart_send(ptr+n_count_send, n_all-n_count_send);
            if(len < 0){
                std::stringstream ss;
                ss << "wrtie data error, write ret=" << len << " all_length=" << n_all 
                    << " sended=" << n_count_send << " time=" << getCurTime() - t_start << "ms";
                whenError(ss.str());
            }
            n_count_send += len;
            if(len == 0) usleep(100);
        }
        if(n_count_send < n_all || getCurTime() - t_start >= _write_timeout){
            std::stringstream ss;
            ss << "wrtie data error, all_length=" << n_all << " sended=" << n_count_send << " time=" << getCurTime() - t_start << "ms";
            whenError(ss.str());
        }
        return n_count_send;
    }
    std::string Serial::read(int length){
        std::vector<char> buffer(length);
        int n_count_read = 0;
        int64_t t_start = getCurTime();
        while(n_count_read < length && getCurTime() - t_start < _read_timeout){
            int len = uart_recv(buffer.data()+n_count_read, 1);
            if(len < 0){
                std::stringstream ss;
                ss << "read data error, read ret=" << len << " all_length=" << length << " readed=" << n_count_read 
                    << " fd=" << _fd << " time=" << getCurTime() - t_start << "ms";
                whenError(ss.str());
            }
            n_count_read += len;
            if(len == 0) usleep(100);
        }
        if(n_count_read < length || getCurTime() - t_start >= _write_timeout){
            std::stringstream ss;
            ss << "read data error, all_length=" << length << " readed=" << n_count_read << " time=" << getCurTime() - t_start << "ms"
                    << ", curTime=" << getCurTime() << ", startTime=" << t_start;
            whenError(ss.str());
        }
        return std::string(buffer.data(), n_count_read);
    }

    std::string Serial::readline(int length){
        const int kMaxLineLength = 1024;
        length = std::min(length, kMaxLineLength);
        std::vector<char> datas(length, 0);
        char* ptr = datas.data();
        int n_count_read = 0, n_all = length;
        int64_t t_start = getCurTime();
        while(n_count_read < n_all && getCurTime() - t_start < _read_timeout){
            int len = uart_recv(ptr+n_count_read, 1);
            if(len < 0){
                std::stringstream ss;
                ss << "read data error, read ret=" << len << " all_length=" << n_all << " readed=" << n_count_read 
                    << " time=" << getCurTime() - t_start << "ms"
                    << ", curTime=" << getCurTime() << ", startTime=" << t_start;
                whenError(ss.str());
            }
            if(ptr[n_count_read] == '\n'){ n_count_read += len; break;}
            n_count_read += len;
            if(len == 0) usleep(100);
        }
        if(getCurTime() - t_start >= _write_timeout){
            std::stringstream ss;
            ss << "read data error, all_length=" << n_all << " readed=" << n_count_read << " time=" << getCurTime() - t_start << "ms";
            whenError(ss.str());
        }
        return std::string(ptr, n_count_read);
    }



    int Serial::uart_open(const char *pathname){
        _fd = open(pathname, O_RDWR|O_NOCTTY);
        if (-1 == _fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
            std::cout << "[HM_RTK] Serial port opened successfully: " << pathname << std::endl;
        if(isatty(STDIN_FILENO)==0)
            std::cout << "[HM_RTK] Standard input is not a terminal device" << std::endl;
        else
            std::cout << "[HM_RTK] Terminal device check passed" << std::endl;
        return _fd;
    }
    int Serial::uart_set(int nSpeed, int nBits, char nEvent, int nStop){
        struct termios newtio,oldtio;
        if  ( tcgetattr( _fd,&oldtio)  !=  0) {
            perror("SetupSerial 1");
            std::cerr << "[HM_RTK ERROR] tcgetattr failed with code: " << tcgetattr(_fd, &oldtio) << std::endl;
            return -1;
        }
        bzero( &newtio, sizeof( newtio ) );
        newtio.c_cflag  |=  CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch( nBits )
        {
            case 7:
            newtio.c_cflag |= CS7;
            break;
            case 8:
            newtio.c_cflag |= CS8;
            break;
        }
        switch( nEvent )
        {
            case 'o':
            case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
            case 'e':
            case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
            case 'n':
            case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
            default:
            break;
        }


        switch( nSpeed ) {
            case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
            case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
            case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
            case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
            case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
            default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        }
        if( nStop == 1 )
            newtio.c_cflag &=  ~CSTOPB;
        else if ( nStop == 2 )
            newtio.c_cflag |=  CSTOPB;
        newtio.c_cc[VTIME]  = 0;
        newtio.c_cc[VMIN] = 0;
        tcflush(_fd,TCIFLUSH);

        if((tcsetattr(_fd,TCSANOW,&newtio))!=0)
        {
            perror("com set error");
            return -1;
        }
        std::cout << "[HM_RTK] Serial port configuration completed" << std::endl;
        return 0;
    }
    int Serial::uart_close(){
        assert(_fd);
        ::close(_fd);
        _fd = 0;
        std::cout << "[HM_RTK] Serial port closed: " << description << std::endl;
        description.clear();

        return 0;
    }
    int Serial::uart_send(const char *send_buffer,int length){
        std::unique_lock<std::mutex> lock(fd_mutex);
        length=::write(_fd,send_buffer,length*sizeof(unsigned char));
	    return length;
    }
    int Serial::uart_recv(char* recv_buffer,int length){
        std::unique_lock<std::mutex> lock(fd_mutex);
        length=::read(_fd,recv_buffer,length);
        return length;
    }

}