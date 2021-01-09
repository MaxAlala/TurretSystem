/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Serial_Rs232.h
 * Author: sol
 *
 * Created on January 2, 2021, 9:05 AM
 */

#ifndef SERIAL_RS232_H
#define SERIAL_RS232_H
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

class Serial_Rs232 {
public:
    ~Serial_Rs232();
    Serial_Rs232(const Serial_Rs232&) = delete;
    void operator=(const Serial_Rs232&) = delete;
    static Serial_Rs232 * getInstance(const std::string port = "/dev/ttyUSB1");
    void write(const std::string& message);
    bool read(std::string& message);
private:
    void changePort(const std::string& port);
    std::string port;
    Serial_Rs232( const std::string & port);
    static Serial_Rs232 * serial_Rs232_instance;
    LibSerial::SerialPort serial_port;
    LibSerial::SerialStream serial_stream;
    // Set the baud rates.
};

#endif /* SERIAL_RS232_H */