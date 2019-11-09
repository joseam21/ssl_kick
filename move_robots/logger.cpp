#include "logger.h"
template <class T>
T Log(std::string identifier, T data, bool use_file){
    if(use_file){
        std::string filename = identifier+".dat";
        if(log_fstreams.count(filename) < 1){
            log_fstreams[filename];
            log_fstreams[filename].open(filename,std::fstream::trunc | std::fstream::out | std::fstream::binary);
        }
        log_fstreams[filename] << data;
    }else{
        std::cout << "Logging Data: " << identifier << std::endl << "   " << data << std::endl;
    }
}
