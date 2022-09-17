#include <stdio.h>
#include <string.h>

struct imu_xyz{
    double x;
    double y;
    double z;
};

struct imu_temp{
    double temp;
};

struct imu_xyz *imu_get_accel_xyz(){
    struct imu_xyz *imu_accel_xyz=NULL;
    return imu_accel_xyz;
}

int main(){
    return 0;
}