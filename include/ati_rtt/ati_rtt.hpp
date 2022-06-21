#ifndef _ATI_RTT_H
#define _ATI_RTT_H

#include <comedilib.h>

#include <kdl/frames.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/os/TimeService.hpp>

#include <geometry_msgs/WrenchStamped.h>

#include <ati_rtt/ftconfig.h>

class ati_rtt : public RTT::TaskContext {

public:

  enum Errno { ESUCCESS, EFAILURE };

private:

  struct Data{
  
    RTT::OutputPort< geometry_msgs::WrenchStamped > port_out_ati;

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
  ati_rtt( const std::string& name = "ati" );
  
  //! Destructor
  ~ati_rtt();
  
  bool setDeviceFile( const std::string& devicefile );
  bool configureSensor( const std::string& name, 
			const std::vector<int> channels,
			const std::string& calib_file );
			


  bool configureHook();
  bool startHook(){}
  void updateHook();
  void stopHook(){}
  void cleanupHook();
    
  //! Read without tool compenstation
  /**
   * @param[out] wrench  wrench data from sensor
   */
  ati_rtt::Errno read( Data* data, float ft[6] );
  
  //! Set bias
  /**
   * This method computes bias wrench and updates bias value. It
   * compensates tool mass, thus need tool transform. 
   */
  bool zero( std::string& name );

};

#endif  //_ATI_RTT_H
