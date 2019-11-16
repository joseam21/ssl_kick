#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>
#include <mutex>

static std::unordered_map<std::string, std::ofstream> log_fstreams;
static std::mutex mtx_logger;

template<typename T>
static T Log(std::string identifier, T data){
    mtx_logger.lock();
    std::string filename = identifier+".dat";
    if(log_fstreams.count(filename) < 1){
        log_fstreams[filename];
        log_fstreams[filename].open(filename);
    }
    //log_fstreams[filename].write(reinterpret_cast<char *>(&data),sizeof(data));
    log_fstreams[filename] << data;
    mtx_logger.unlock();
}

#endif
