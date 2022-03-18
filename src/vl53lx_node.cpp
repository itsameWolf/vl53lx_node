#include <ros/ros.h>
#include <ros/console.h>

#include "vl53lx_node/vl53lx_MultiRangingData.h"
#include "vl53lx_node/vl53lx_TargetRangeData.h"

#include "vl53lx_node/vl53lx_StartRanging.h"
#include "vl53lx_node/vl53lx_StopRanging.h"

#include <vl53lx_api/vl53lx_api.h>
#include <vl53lx_api/vl53lx_platform.h>

VL53LX_Dev_t                   dev;
VL53LX_DEV                     Dev = &dev;

int status;
bool ranging_flag = false; 

float polling_ratio = 4;

vl53lx_node::vl53lx_MultiRangingData MultiRangingDataMSG;

bool startRanging(vl53lx_node::vl53lx_StartRanging::Request &req,
                  vl53lx_node::vl53lx_StartRanging::Response &res);

bool stopRanging(vl53lx_node::vl53lx_StopRanging::Request &req,
                  vl53lx_node::vl53lx_StopRanging::Response &res);   

char* I2c_address = (char*)"/dev/i2c-1";

void RangingLoop(void);


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "vl53lx_node");
  ros::NodeHandle n;
  ROS_INFO("vl53lx node initialised");

  ros::ServiceServer start_ranging = n.advertiseService("start_ranging", startRanging);
  ROS_INFO("start_ranging service server started");
  ros::ServiceServer stop_ranging = n.advertiseService("stop_ranging", stopRanging);
  ROS_INFO("stop_ranging service server started");



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

bool startRanging(vl53lx_node::vl53lx_StartRanging::Request &req,
                  vl53lx_node::vl53lx_StartRanging::Response &res)
{
  Dev->I2cDevAddr = 0x29;
  Dev->fd = VL53LX_i2c_init(I2c_address, Dev->I2cDevAddr); //choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3
  ROS_INFO("Dev->fd: %d", Dev->fd);

  if (Dev->fd<0) {
      ROS_INFO("Failed to init");
      ranging_flag = false;
      return false;
  }

  if(status){
      ROS_INFO("VL53LX_StartMeasurement failed: error = %d", status);
      ranging_flag = false;
      return false;
  }

  uint8_t byteData;
  uint16_t wordData;
  VL53LX_RdByte(Dev, 0x010F, &byteData);
  ROS_INFO("VL53LX Model_ID: %02X", byteData);
  if(byteData != 0xea) {
      ROS_INFO("WARNING: Model Id is not 0xea, which it ought to be!");
  }
  VL53LX_RdByte(Dev, 0x0110, &byteData);
  ROS_INFO("VL53LX Module_Type: %02X", byteData);
  if(byteData != 0xaa) {
      ROS_INFO("WARNING: Module type is not 0xaa, which it ought to be!");
  }
  VL53LX_RdWord(Dev, 0x010F, &wordData);
  ROS_INFO("VL53LX: %02X\n\r", wordData);
  
  if(status){
      ROS_INFO("VL53LX_StartMeasurement failed: error = %d \n", status);
      ranging_flag = false;
      return false;
  }

  usleep(10000);
  
  ranging_flag = true;
  return true;
}

bool stopRanging(vl53lx_node::vl53lx_StopRanging::Request &req,
                  vl53lx_node::vl53lx_StopRanging::Response &res)
{
  status = VL53LX_StopMeasurement(Dev);
  ROS_INFO("stopped ranging with status: %d", status);
  Dev->fd = VL53LX_i2c_close();
  if (status)
  {
    ranging_flag = true;
    return false;
  }
  else
  {
    ranging_flag = false;
    return true;
  }
}

void RangingLoop(void)
{
  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady=0;
  int no_of_object_found=1;
  //int j;
  ROS_INFO ("Ranging loop starts");
  
  status = VL53LX_WaitDeviceBooted(Dev);
  status = VL53LX_DataInit(Dev);
  status = VL53LX_StartMeasurement(Dev);
  
  if(status){
    printf("VL53LX_StartMeasurement failed: error = %d \n", status);
    while(1);
  }
  do{ // driver polling mode
    // Wait for data to be read, blocking.
    status = VL53LX_GetMeasurementDataReady(Dev,&NewDataReady);                       
    usleep(250000);
    if ((status == VL53LX_ERROR_NONE)&&(NewDataReady==1)){
        // wait for measurement data ready
        status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
        ROS_INFO("Status: %d Objects found=%5d \n ",status, no_of_object_found);
        for(int j=0;j<no_of_object_found;j++){
            ROS_INFO("status=%d, D=%5dmm, S=%7dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
               pMultiRangingData->RangeData[j].RangeStatus,
               pMultiRangingData->RangeData[j].RangeMilliMeter,
               pMultiRangingData->RangeData[j].SigmaMilliMeter,
               pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
               pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
        }
        // clear interupt and start new measurement
        if (status==0)
        {
          status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
    }
    ros::spinOnce();
  }
  while (ranging_flag == true);
}