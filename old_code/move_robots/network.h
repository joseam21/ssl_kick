#ifndef NETWORK_H
#define NETWORK_H

bool sendCommand(bool isYellow, float timestamp, int id, float kickspeedx, float kickspeedz, float veltangent, 
    float velnormal, float velangular, bool spinner, bool wheelsspeed, float wheel1, float wheel2, float wheel3, float wheel4);
bool sendReplacement(float x, float y, float dir, int id, bool isYellow);
void getLocation(int robot_id);
#endif // NETWORK_H
