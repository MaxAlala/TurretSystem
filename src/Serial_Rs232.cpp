/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Serial_Rs232.cpp
 * Author: sol
 * 
 * Created on January 2, 2021, 9:05 AM
 */

#include "Serial_Rs232.h"
Serial_Rs232 * Serial_Rs232::serial_Rs232_instance
{
    nullptr
};
Serial_Rs232::Serial_Rs232(const std::string& port) : port{port}
{
    serial_port.Open(port);
    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
}

Serial_Rs232 * Serial_Rs232::getInstance(const std::string port) {
    if (serial_Rs232_instance == nullptr) {
        serial_Rs232_instance = new Serial_Rs232(port);
    } else {
        serial_Rs232_instance->changePort(port);
    }

    return serial_Rs232_instance;
}

Serial_Rs232::~Serial_Rs232() {
    serial_port.Close();
}

void Serial_Rs232::changePort(const std::string& port) {
    if (serial_port.IsOpen())
        serial_port.Close();
    serial_port.Open(port);
    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
}

void Serial_Rs232::write(const std::string& message) {
    serial_port.Write(message);
}

bool Serial_Rs232::read(std::string& message) {
    if (serial_port.GetNumberOfBytesAvailable() != 0)
        serial_port.ReadLine(message);
}
