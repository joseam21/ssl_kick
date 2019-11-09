#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <unordered_map>

std::unordered_map<std::string, std::fstream> log_fstreams;

template <class T>
T Log(std::string identifier, T data, bool use_file);

#endif
