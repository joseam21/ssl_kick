#ifndef NETWORK_H
#define NETWORK_H

bool sendVelocity(int robot_id, double veltangent, double velnormal, double velangular);
void getLocation(int robot_id);
#endif // NETWORK_H
