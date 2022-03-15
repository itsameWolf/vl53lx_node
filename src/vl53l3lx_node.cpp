#include "ros/ros.h"
#include "ros/console.h"

#include "vl53l3lx_node/vl53l3lx_MultiRangingData.h"
#include "vl53l3lx_node/vl53l3lx_TargetRangeData.h"

#include "vl53l3lx_node/vl53l3lx_StartRanging.h"
#include "vl53l3lx_node/vl53l3lx_StopRanging.h"

#include "vl53lx_api.h"
#include "vl53lx_platform.h"

VL53LX_Dev_t                   dev;
VL53LX_DEV                     Dev = &dev;

VL53LX_MultiRangingData_t MultiRangingData;
VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

int status;
bool ranging_flag = 0; 

float polling_ratio = 4;

vl53l3lx_node::vl53l3lx_MultiRangingData MultiRangingDataMSG;

uint8_t NewDataReady = 0;

bool startRanging(vl53l3lx_node::vl53l3lx_StartRanging::Request &req,
                  vl53l3lx_node::vl53l3lx_StartRanging::Response &res);

bool stopRanging(vl53l3lx_node::vl53l3lx_StopRanging::Request &req,
                  vl53l3lx_node::vl53l3lx_StopRanging::Response &res);   

void RangingLoop(void);


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "vl53l3lx_node");

  ros::NodeHandle n;

  ros::ServiceServer start_ranging = n.advertiseService("start_ranging", startRanging);
  ros::ServiceServer stop_ranging = n.advertiseService("stop_ranging", stopRanging);

  while (ros::ok())
  { 
    if (ranging_flag)
    {
      RangingLoop();
    }
    ros::spinOnce();
    //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1./polling_ratio));
  }
  

}

bool startRanging(vl53l3lx_node::vl53l3lx_StartRanging::Request &req,
                  vl53l3lx_node::vl53l3lx_StartRanging::Response &res)
{
  Dev->I2cDevAddr = 0x29;
  Dev->fd = VL53LX_i2c_init("/dev/i2c-1", Dev->I2cDevAddr); //choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3

  if (Dev->fd<0) {
      ROS_INFO("Failed to init\n");
      ranging_flag = 0;
      return 0;
  }
      status = VL53LX_DataInit(Dev);
      status = VL53LX_StartMeasurement(Dev);

  if(status){
      ROS_INFO("VL53LX_StartMeasurement failed: error = %d \n", status);
      ranging_flag = 0;
      return 0;
  }

  uint8_t byteData;
  uint16_t wordData;
  VL53LX_RdByte(Dev, 0x010F, &byteData);
  printf("VL53LX Model_ID: %02X\n\r", byteData);
  if(byteData != 0xea) {
      ROS_INFO("WARNING: Model Id is not 0xea, which it ought to be!\n");
  }
  VL53LX_RdByte(Dev, 0x0110, &byteData);
  printf("VL53LX Module_Type: %02X\n\r", byteData);
  if(byteData != 0xaa) {
      ROS_INFO("WARNING: Module type is not 0xaa, which it ought to be!\n");
  }
  VL53LX_RdWord(Dev, 0x010F, &wordData);
  ROS_INFO("VL53LX: %02X\n\r", wordData);

  ROS_INFO("Ranging loop starts\n");

  status = VL53LX_WaitDeviceBooted(Dev);
  status = VL53LX_DataInit(Dev);
  status = VL53LX_StartMeasurement(Dev);
  
  if(status){
      printf("VL53LX_StartMeasurement failed: error = %d \n", status);
      ranging_flag = 0;
      return 0;
  }
  
  ranging_flag = 1;
  return 1;
}

bool stopRanging(vl53l3lx_node::vl53l3lx_StopRanging::Request &req,
                  vl53l3lx_node::vl53l3lx_StopRanging::Response &res)
{
  status = VL53LX_StopMeasurement(Dev);
  Dev->fd = VL53LX_i2c_close();
  if (status)
  {
    ranging_flag = 1;
    return 0;
  }
  else
  {
    ranging_flag = 0;
    return 1;
  }
}

void RangingLoop(void)
{
  int no_of_object_found = 0,j;
  status = VL53LX_GetMeasurementDataReady(Dev, &NewDataReady);                        
  usleep(250000); // 250 millisecond polling period, could be 1 millisecond.
  if((!status)&&(NewDataReady!=0)){
    status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
    no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
    printf("Count=%5d, ", pMultiRangingData->StreamCount);
    printf("#Objs=%1d ", no_of_object_found);
    for(j=0;j<no_of_object_found;j++){
      if(j!=0)printf("\n                     ");
      ROS_INFO("status=%d, D=%5dmm, S=%7dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
              pMultiRangingData->RangeData[j].RangeStatus,
              pMultiRangingData->RangeData[j].RangeMilliMeter,
              pMultiRangingData->RangeData[j].SigmaMilliMeter,
              pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
              pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
    }
    printf ("\n");
    if (status==0){
      status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
    }
  }
}
