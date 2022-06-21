
#include <errno.h>
#include <string.h>
#include <iomanip>
#include <math.h>

#include <comedi.h>
#include <comedilib.h>

#include <ati_rtt/ati_rtt.hpp>

#include <rtt/Component.hpp>         // use for ORO_CREATE_COMPONENT

ati_rtt::ati_rtt( const std::string& name ):
  RTT::TaskContext( name ),

  set_device_file_mtd( "setDeviceFile",
		       &ati_rtt::setDeviceFile,
		       this,
		       RTT::ClientThread ),
  configure_sensor_mtd( "configureSensor",
			&ati_rtt::configureSensor,
			this,
			RTT::ClientThread ),
  zero_mtd( "zero",
	    &ati_rtt::zero,
	    this,
	    RTT::ClientThread ){

  addOperation( set_device_file_mtd ).doc( "Set the comedi device file." );
  addOperation( configure_sensor_mtd ).doc( "Configure ati_rtt sensor." );
  addOperation( zero_mtd ).doc( "Zero ati_rtt sensor." );
  
}

bool ati_rtt::setDeviceFile( const std::string& s ){
  devicefile = s;
  return true;
}

bool ati_rtt::configureSensor( const std::string& name,
			   const std::vector<int> channels,
			   const std::string& calib_file ){

  frame_id = "/"+name+"/tcp";
  
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
    std::cout << "ati_rtt::SetCalibFile: couldn't load the calibration file " 
	      << buffer << std::endl;
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

bool ati_rtt::configureHook(){

  Device = comedi_open( devicefile.c_str() );
  Subdevice = 0;
    
  if( Device == NULL ){
    RTT::log(RTT::Error) << "ati_rtt::configureHook: Failed to open device: " 
			 << devicefile << RTT::endlog(); 
    return false;
  }
  
  std::cout << "Board Name: " << comedi_get_board_name( Device ) << std::endl;
  
  int nsubdev = comedi_get_n_subdevices( Device );
  int flag = comedi_get_subdevice_flags( Device, Subdevice );
  /*
  if( flag & SDF_BUSY )
    { std::cout << "The subdevice is busy performing an asynchronous command." << std::endl; }
  if( flag & SDF_BUSY_OWNER )
    { std::cout << "The subdevice is busy, and the command it is running was started by the current process." << std::endl; }
  if( flag & SDF_LOCKED )
    { std::cout << "The subdevice has been locked by comedi_lock." << std::endl; }
  if( flag & SDF_LOCK_OWNER )
    { std::cout << "The subdevice is locked, and was locked by the current process." << std::endl; }
  if( flag & SDF_MAXDATA )
    { std::cout << "The maximum data value for the subdevice depends on the channel." << std::endl; }
  if( flag & SDF_FLAGS )
    { std::cout << "The subdevice flags depend on the channel (unfinished/broken support in library)." << std::endl; }
  if( flag & SDF_RANGETYPE )
    { std::cout << "The range type depends on the channel." << std::endl; }
  if( flag & SDF_CMD )
    { std::cout << "The subdevice supports asynchronous commands." << std::endl; }
  if( flag & SDF_SOFT_CALIBRATED )
    { std::cout << "The subdevice relies on the host to do calibration in software." << std::endl; }
  if( flag & SDF_READABLE )
    { std::cout << "The subdevice can be read (e.g. analog input)." << std::endl; }
  if( flag & SDF_WRITABLE )
    { std::cout << "The subdevice can be written to (e.g. analog output)." << std::endl; }
  if( flag & SDF_INTERNAL )
    { std::cout << "The subdevice does not have externally visible lines." << std::endl; }
  if( flag & SDF_GROUND )
    { std::cout << "The subdevice supports analog reference AREF_GROUND." << std::endl; }
  if( flag & SDF_COMMON )
    { std::cout << "The subdevice supports analog reference AREF_COMMON." << std::endl; }
  if( flag & SDF_DIFF )
    { std::cout << "The subdevice supports analog reference AREF_DIFF." << std::endl; }
  if( flag & SDF_OTHER )
    { std::cout << "The subdevice supports analog reference AREF_OTHER." << std::endl; }
  if( flag & SDF_DITHER )
    { std::cout << "The subdevice supports dithering (via the CR_ALT_FILTER chanspec flag)." << std::endl; }
  if( flag & SDF_DEGLITCH )
    { std::cout << "The subdevice supports deglitching (via the CR_ALT_FILTER chanspec flag)." << std::endl; }
  if( flag & SDF_RUNNING )
    { std::cout << "An asynchronous command is running." << std::endl; }
  if( flag & SDF_LSAMPL )
    { std::cout << "The subdevice uses the 32-bit lsampl_t type instead of the 16-bit sampl_t for asynchronous command data." << std::endl; }
  if( flag & SDF_PACKED )
    { std::cout << "The subdevice uses bitfield samples for asynchronous command data, one bit per channel (otherwise it uses one sampl_t or lsampl_t per channel). Commonly used for digital subdevices." << std::endl; }
  */

  
  for( int i=0; i<comedi_get_n_channels( Device, Subdevice ); i++ ){
    if( comedi_apply_calibration( Device, Subdevice, i, 1, AREF_DIFF, NULL ) != 0 ){
      RTT::log(RTT::Error) << "failed to calibrate channel " << i
			   << RTT::endlog();
    }
  }
  std::cout << "Board has been calibrated" << std::endl;
  
    /*
  int nchan = comedi_get_n_channels( Device, Subdevice );
  for( int j=0; j<nchan; j++ ){
    std::cout << "Subdevice: " << Subdevice << " "
	      << "Channel: "   << std::setw(2) << j << " "
	      << "Ranges: ";
    int nranges = comedi_get_n_ranges( Device, Subdevice, j );
    for( int k=0; k<nranges; k++ ){

      comedi_range* range = comedi_get_range( Device, Subdevice, j, k );
      std::cout << "(" << range->min 
		<< ", " << range->max << ") ";
    }
    std::cout << std::endl;
  }
    */
  return true;

}

void ati_rtt::cleanupHook(){
  if( Device != NULL ){
    if( comedi_close( Device ) == -1 )
      { std::cout << "ati_rtt::Closed: Failed to close." << std::endl; }
  }
}

ati_rtt::~ati_rtt()
{}


bool ati_rtt::zero( std::string& name ){

  double zero[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  for( size_t i=0; i<1000; i++ ){
    float ft[6];
    read( sensors[name], ft );
    for( int j=0; j<6; j++ )
      { zero[j] += ft[j]; }
  }

  for( int j=0; j<6; j++ )
    { sensors[name]->bias[j] = zero[j]/1000; }

  return true;
}

void ati_rtt::updateHook(){

  std::map<std::string,Data*>::iterator it;
  for( it=sensors.begin(); it!=sensors.end(); it++ ){
    float ft[6];
    if( read( it->second, ft ) == ati_rtt::ESUCCESS ){
      geometry_msgs::WrenchStamped w;
      w.wrench.force.x = ft[0] - it->second->bias[0];
      w.wrench.force.y = ft[1] - it->second->bias[1];
      w.wrench.force.z = ft[2] - it->second->bias[2];
      w.wrench.torque.x = ft[3] - it->second->bias[3];
      w.wrench.torque.y = ft[4] - it->second->bias[4];
      w.wrench.torque.z = ft[5] - it->second->bias[5];
      w.header.frame_id = frame_id;
      w.header.stamp = ros::Time::now();
      it->second->port_out_ati.write( w );
    }
  }

}

ati_rtt::Errno ati_rtt::read( Data* data, float ft[6] ){

  float voltages[6];
  std::map<char,int>::const_iterator it;

  for( it=data->channels.begin(); it!=data->channels.end(); it++ ){
    lsampl_t sample = 0;
    int cid = it->second;;
    int err;
    
    err = comedi_data_read( Device, Subdevice, cid, 1, AREF_DIFF, &sample );
    //std::cout << std::setw(5) << "(" << err << "):" << std::setw(6) << sample;
    //if( (cid+1) % 6 == 0 ) std::cout << std::endl;

    //const char* string = comedi_strerror( comedi_errno() );
    //std::cout << "ati_rtt::Read: " << string << std::endl;
    //std::cout << " (" << comedi_get_maxdata( Device, Subdevice, cid ) << ")";
    //std::cout << Device << " " << Subdevice << std::endl;
    //std::cout << comedi_get_range( Device, Subdevice, cid, 1 )->min << " "
    //	      << comedi_get_range( Device, Subdevice, cid, 1 )->max << std::endl;
    //std::cout << comhttp://ici.radio-canada.ca/edi_get_range( Device, Subdevice, cid, 1 ) << std::endl;
    //std::cout << comedi_get_maxdata( Device, Subdevice, cid ) << std::endl;
    
    if( err == 1 ) {
      double val=comedi_to_phys( sample, 
				 comedi_get_range( Device, Subdevice, cid, 1 ),
				 comedi_get_maxdata( Device, Subdevice, cid ) );

      
      if( isnan(val) ){
	const char* string = comedi_strerror( comedi_errno() );
	std::cout << "ati_rtt::Read: " << string << std::endl;
	return ati_rtt::EFAILURE;
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
      std::cout << "ati_rtt::Read: " << string << std::endl;
      return ati_rtt::EFAILURE;
    }
  }
  //std::cout << std::endl;

  ConvertToFT( data->cal, voltages, ft );
  
  return ati_rtt::ESUCCESS;

}

ORO_CREATE_COMPONENT( ati_rtt )

