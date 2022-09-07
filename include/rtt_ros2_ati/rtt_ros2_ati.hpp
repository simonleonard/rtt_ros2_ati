#ifndef _RTT_ROS2_ATI_H_
#define _RTT_ROS2_ATI_H_

#include <comedilib.h>

#include <kdl/frames.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/os/TimeService.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <rtt_ros2_node/rtt_ros2_node.hpp>

#include <rtt_ros2_ati/ftconfig.h>

class rtt_ros2_ati : public RTT::TaskContext {

public:

  enum Errno { ESUCCESS, EFAILURE };

private:

  struct Data{
  
    RTT::OutputPort< geometry_msgs::msg::WrenchStamped > port_out_ati;
    RTT::OutputPort< geometry_msgs::msg::WrenchStamped > port_out_ati_raw;

    std::string calibfile;
    Calibration *cal;

    std::map<char,int> channels;
    
    float bias[6];

  };

  std::string devicefile;
  std::string frame_id;
  comedi_t* Device;
  int Subdevice;
    
  std::map<std::string,Data*> sensors;

  RTT::Operation<bool(std::string)> set_device_file_mtd;
  RTT::Operation<bool(std::string,
		      std::vector<int>,
		      std::string)> configure_sensor_mtd;
  RTT::Operation<bool(std::string)> zero_mtd;

public:
  
  //! \brief Constructors
  /** 
   * @param name  task name
   */
  rtt_ros2_ati( const std::string& name = "ati" );
  
  //! Destructor
  ~rtt_ros2_ati();
  
  bool setDeviceFile( const std::string& devicefile );
  bool configureSensor( const std::string& name, 
			const std::vector<int> channels,
			const std::string& calib_file );
			


  bool configureHook();
  bool startHook(){ return true;}
  void updateHook();
  void stopHook(){}
  void cleanupHook();
    
  //! Read without tool compenstation
  /**
   * @param[out] wrench  wrench data from sensor
   */
  rtt_ros2_ati::Errno read( Data* data, float ft[6] );
  
  //! Set bias
  /**
   * This method computes bias wrench and updates bias value. It
   * compensates tool mass, thus need tool transform. 
   */
  bool zero( const std::string& name );

};

#endif  //_RTT_ROS2_ATI_H
