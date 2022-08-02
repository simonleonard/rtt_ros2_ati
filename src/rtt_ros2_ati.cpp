
#include <errno.h>
#include <string.h>
#include <iomanip>
#include <math.h>

#include <comedi.h>
#include <comedilib.h>

#include <rtt/internal/GlobalService.hpp>

#include <rtt_ros2_ati/rtt_ros2_ati.hpp>

#include <rtt/Component.hpp>         // use for ORO_CREATE_COMPONENT

rtt_ros2_ati::rtt_ros2_ati( const std::string& name ):
  RTT::TaskContext( name ),
  set_device_file_mtd( "setDeviceFile", &rtt_ros2_ati::setDeviceFile, this, RTT::ClientThread ),
  configure_sensor_mtd( "configureSensor", &rtt_ros2_ati::configureSensor, this, RTT::ClientThread ),
  zero_mtd( "zero", &rtt_ros2_ati::zero, this, RTT::ClientThread ){
  
  RTT::Logger::In(this->getName());

  RTT::Service::shared_ptr global_ros =
    RTT::internal::GlobalService::Instance()->getService("ros");
  RTT::OperationCaller<bool()> create_node =
    global_ros->getOperation("create_node");
  create_node.ready();
  create_node();

  addOperation( set_device_file_mtd ).doc( "Set the comedi device file." );
  addOperation( configure_sensor_mtd ).doc( "Configure rtt_ros2_ati sensor." );
  addOperation( zero_mtd ).doc( "Zero rtt_ros2_ati sensor." );
  
}

bool rtt_ros2_ati::setDeviceFile( const std::string& s ){
  devicefile = s;
  return true;
}

bool rtt_ros2_ati::configureSensor( const std::string& name,
				    const std::vector<int> channels,
				    const std::string& calib_file ){
  
  frame_id = "/"+name;/*+"/tcp";*/
  
  Data* data = new Data;
  addPort( name, data->port_out_ati );
  data->channels['x'] = channels[0];
  data->channels['y'] = channels[1];
  data->channels['z'] = channels[2];
  data->channels['X'] = channels[3];
  data->channels['Y'] = channels[4];
  data->channels['Z'] = channels[5];

  for( int i=0; i<6; i++ )
    { data->bias[i] = 0.0; }

  data->calibfile = calib_file;
  char buffer[1024];
  strcpy(buffer, calib_file.c_str() );
  data->cal = createCalibration( buffer, 1 );
  if( data->cal == NULL ){ 
    RTT::log(RTT::Error) << "rtt_ros2_ati::SetCalibFile: couldn't load the calibration file " 
			 << buffer <<  RTT::endlog();
    return false;
  }

  char funits[] = "N";
  char tunits[] = "N-m";
  SetForceUnits( data->cal, funits );
  SetTorqueUnits( data->cal, tunits );

  std::pair<std::string,Data*> pair( name, data );
  sensors.insert( pair );

  return true;
}

bool rtt_ros2_ati::configureHook(){

  Device = comedi_open( devicefile.c_str() );
  Subdevice = 0;
    
  if( Device == NULL ){
    RTT::log(RTT::Error) << "rtt_ros2_ati::configureHook: Failed to open device: " 
			 << devicefile
			 << RTT::endlog(); 
    return false;
  }
  
  RTT::log(RTT::Info) << "Board Name: " << comedi_get_board_name( Device ) << RTT::endlog();
  
  //int nsubdev = comedi_get_n_subdevices( Device );
  int flag = comedi_get_subdevice_flags( Device, Subdevice );

  if( flag & SDF_BUSY )
    { RTT::log(RTT::Info) << "The subdevice is busy performing an asynchronous command."
			  << RTT::endlog(); }
  if( flag & SDF_BUSY_OWNER )
    { RTT::log(RTT::Info) << "The subdevice is busy, and the command it is running was started by the current process."
			  << RTT::endlog(); }
  if( flag & SDF_LOCKED )
    { RTT::log(RTT::Info) << "The subdevice has been locked by comedi_lock."
			  << RTT::endlog(); }
  if( flag & SDF_LOCK_OWNER )
    { RTT::log(RTT::Info) << "The subdevice is locked, and was locked by the current process."
			  << RTT::endlog(); }
  if( flag & SDF_MAXDATA )
    { RTT::log(RTT::Info) << "The maximum data value for the subdevice depends on the channel."
			  << RTT::endlog(); }
  if( flag & SDF_FLAGS )
    { RTT::log(RTT::Info) << "The subdevice flags depend on the channel (unfinished/broken support in library)."
			  << RTT::endlog(); }
  if( flag & SDF_RANGETYPE )
    { RTT::log(RTT::Info) << "The range type depends on the channel."
			  << RTT::endlog(); }
  if( flag & SDF_CMD )
    { RTT::log(RTT::Info) << "The subdevice supports asynchronous commands."
			  << RTT::endlog(); }
  if( flag & SDF_SOFT_CALIBRATED )
    { RTT::log(RTT::Info) << "The subdevice relies on the host to do calibration in software."
			  << RTT::endlog(); }
  if( flag & SDF_READABLE )
    { RTT::log(RTT::Info) << "The subdevice can be read (e.g. analog input)."
			  << RTT::endlog(); }
  if( flag & SDF_WRITABLE )
    { RTT::log(RTT::Info) << "The subdevice can be written to (e.g. analog output)."
			  << RTT::endlog(); }
  if( flag & SDF_INTERNAL )
    { RTT::log(RTT::Info) << "The subdevice does not have externally visible lines."
			  << RTT::endlog(); }
  if( flag & SDF_GROUND )
    { RTT::log(RTT::Info) << "The subdevice supports analog reference AREF_GROUND."
			  << RTT::endlog(); }
  if( flag & SDF_COMMON )
    { RTT::log(RTT::Info) << "The subdevice supports analog reference AREF_COMMON."
			  << RTT::endlog(); }
  if( flag & SDF_DIFF )
    { RTT::log(RTT::Info) << "The subdevice supports analog reference AREF_DIFF."
			  << RTT::endlog(); }
  if( flag & SDF_OTHER )
    { RTT::log(RTT::Info) << "The subdevice supports analog reference AREF_OTHER."
			  << RTT::endlog(); }
  if( flag & SDF_DITHER )
    { RTT::log(RTT::Info) << "The subdevice supports dithering (via the CR_ALT_FILTER chanspec flag)."
			  << RTT::endlog(); }
  if( flag & SDF_DEGLITCH )
    { RTT::log(RTT::Info) << "The subdevice supports deglitching (via the CR_ALT_FILTER chanspec flag)."
			  << RTT::endlog(); }
			  
  if( flag & SDF_RUNNING )
    { RTT::log(RTT::Info) << "An asynchronous command is running."
			  << RTT::endlog(); }

  if( flag & SDF_LSAMPL )
    { RTT::log(RTT::Info) << "The subdevice uses the 32-bit lsampl_t type instead of the 16-bit sampl_t for asynchronous command data."
			  << RTT::endlog(); }

  if( flag & SDF_PACKED )
    { RTT::log(RTT::Info) << "The subdevice uses bitfield samples for asynchronous command data, one bit per channel (otherwise it uses one sampl_t or lsampl_t per channel). Commonly used for digital subdevices."
			  << RTT::endlog(); }

  for( int c=0; c<comedi_get_n_channels( Device, Subdevice ); c++ ){
    for( int r=0; r<comedi_get_n_ranges( Device, Subdevice, c ); r++ ){
      if( comedi_apply_calibration( Device, Subdevice, c, r, AREF_DIFF, NULL ) != 0 ){
	RTT::log(RTT::Error) << "failed to calibrate channel " << c
			     << RTT::endlog();
      }
    }
  }
  std::cout << "Board has been calibrated" << std::endl;

  return true;

}

void rtt_ros2_ati::cleanupHook(){
  if( Device != NULL ){
    if( comedi_close( Device ) == -1 )
      { RTT::log(RTT::Info) << "rtt_ros2_ati::Closed: Failed to close."
			    << RTT::endlog(); }			    
  }
}

rtt_ros2_ati::~rtt_ros2_ati(){}

bool rtt_ros2_ati::zero( const std::string& name ){

  double zero[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  for( size_t i=0; i<1000; i++ ){
    float ft[6];
    read( sensors[name], ft );
    for( int j=0; j<6; j++ )
      { zero[j] -= ft[j]; }
  }

  for( int j=0; j<6; j++ )
    { sensors[name]->bias[j] = zero[j]/1000; }

  return true;
}

void rtt_ros2_ati::updateHook(){

  std::map<std::string,Data*>::iterator it;
  for( it=sensors.begin(); it!=sensors.end(); it++ ){
    float ft[6];
    if( read( it->second, ft ) == rtt_ros2_ati::ESUCCESS ){
      geometry_msgs::msg::WrenchStamped w;
      w.wrench.force.x = ft[0] + it->second->bias[0];
      w.wrench.force.y = ft[1] + it->second->bias[1];
      w.wrench.force.z = ft[2] + it->second->bias[2];
      w.wrench.torque.x = ft[3] + it->second->bias[3];
      w.wrench.torque.y = ft[4] + it->second->bias[4];
      w.wrench.torque.z = ft[5] + it->second->bias[5];
      w.header.frame_id = frame_id;
      //w.header.stamp = ros::Time::now();
      it->second->port_out_ati.write( w );
    }
  }

}

rtt_ros2_ati::Errno rtt_ros2_ati::read( Data* data, float ft[6] ){

  float voltages[6];
  std::map<char,int>::const_iterator it;

  for( it=data->channels.begin(); it!=data->channels.end(); it++ ){
    lsampl_t sample = 0;
    int cid = it->second;;
    int err;
    
    err = comedi_data_read( Device, Subdevice, cid, 1, AREF_DIFF, &sample );
    
    if( err == 1 ) {
      double val=comedi_to_phys( sample, 
				 comedi_get_range( Device, Subdevice, cid, 1 ),
				 comedi_get_maxdata( Device, Subdevice, cid ) );

      if( isnan(val) ){
	const char* string = comedi_strerror( comedi_errno() );
	RTT::log(RTT::Error) << "rtt_ros2_ati::Read: NaN"
			     << RTT::endlog();
	return rtt_ros2_ati::EFAILURE;
      }
      else{
	switch( it->first ){
	case 'x': voltages[0] = val; break;
	case 'y': voltages[1] = val; break;
	case 'z': voltages[2] = val; break;
	case 'X': voltages[3] = val; break;
	case 'Y': voltages[4] = val; break;
	case 'Z': voltages[5] = val; break;
	default:              break;
	}
      }
    }
    else{
      const char* string = comedi_strerror( comedi_errno() );
      RTT::log(RTT::Error) << "rtt_ros2_ati::Read: " << string
			   << RTT::endlog();
      return rtt_ros2_ati::EFAILURE;
    }
  }

  ConvertToFT( data->cal, voltages, ft );
  
  return rtt_ros2_ati::ESUCCESS;

}

ORO_CREATE_COMPONENT( rtt_ros2_ati )
