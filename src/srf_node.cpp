#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Illuminance.h"
#include "serial/serial.h"
#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <iomanip>
#include <sensor_filters/MedianFilterSimple.h>
#include <sensor_filters/LowPassSimple.h>

// I2C Converter Docu http://www.robot-electronics.co.uk/htm/usb_i2c_tech.htm
#define I2C_AD1 0x55 // Read/Write single or multiple bytes for 1 byte addressed devices (the majority of devices will use this one)
#define I2C_USB 0x5A // A range of commands to the USB-I2C module, generally to improve selected communications or provide analogue/digital I/O

// SRF08 Datasheet http://www.cs.york.ac.uk/micromouse/Docs/SRF08UltraSonicRanger.pdf
#define SRF_VERSION_AND_COMMAND_REGISTER  	0x00
#define SRF08_LIGHT_AND_GAIN_REGISTER  		0x01
#define SRF_RANGE_REGISTER  		        0x02

#define SRF08_DEFAULT_I2C_ADDRESS		0xe0
#define SRF02_DEFAULT_I2C_ADDRESS               0xe0

using namespace sensor_filters;
using namespace std;
using namespace std_msgs;

namespace srf
{

enum MeasureUnit
{
  UNIT_INCH = 0x50, UNIT_CM = 0x51, UNIT_MICROSEC = 0x52
};

/***
 * General Class for Accessing SRF08 I2C range sensors over USB IC2 Converter
 * author: hrabia
 */
class SRFUltrasonsicRangeFinder
{
public:

  SRFUltrasonsicRangeFinder(boost::shared_ptr<serial::Serial> serial, int i2c_address = SRF08_DEFAULT_I2C_ADDRESS) :
      serial_(serial), i2c_address_(i2c_address), version_(0)
  {
    version_ = readVersion();
  }

  uint8_t readVersion()
  {
    uint8_t version = 0xff;

    readCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, &version, 1);
    if(version == 0xff)
    {
      string err_msg("Could not determine sonar version");
      ROS_ERROR_STREAM(err_msg.c_str());
      throw err_msg;
    }
    return version;
  }

  /**
   * Permanently change the i2c address of the device,
   * only one device should be connected to the i2c bus
   * during this operation
   *
   * Only even numbers are allowed e.g.
   * 0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF4, 0xF6, 0xF8, 0xFA, 0xFC, 0xFE
   *
   * @param new_address
   * @return true on success
   */
  bool changeI2CAddress(int new_address)
  {
    //check if the i2c address is even or odd
    if(new_address %2 != 0){
      ROS_ERROR_STREAM("Only even i2c addresses are allowed for this device");
      return false;
    }

    int old_address = i2c_address_;
    try
    {
      //write following command order for changing the address to register 0x00 : 0xA0 , 0xAA, 0xA5
      writeCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, 0xA0);
      writeCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, 0xAA);
      writeCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, 0xA5);
      writeCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, new_address);

      i2c_address_ = new_address;

      //test if address change for successful

      readVersion();

      ROS_INFO_STREAM("Address changed to: 0x" << std::hex  << i2c_address_);

      return true;

    }
    catch (...)
    {
      i2c_address_  = old_address;
      ROS_ERROR_STREAM("Address change failed, keeping old address: 0x" << std::hex  << i2c_address_);
      return false;
    }
  }

  bool requestRanging(MeasureUnit unit)
    {
      try
      {
        //request ranging
        writeCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, unit);
        return true;
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Ranging request failed");
        return false;
      }
    }

  bool updateMeasurement(int& light, int& distance)
  {
    try
    {

      //wait for version --> device ready
      uint8_t version = 0xff;
      do
      {
        //request result distance and light
        uint8_t buffer[4] = {0};
        readCommand(I2C_AD1, i2c_address_, SRF_VERSION_AND_COMMAND_REGISTER, buffer, sizeof(buffer));

        version = buffer[0];
        light = buffer[1];
        distance = (buffer[2] << 8) | buffer[3];
      } while (version != version_ );

      // valid distance is never 0
      return distance != 0;
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Update failed");
      return false;
    }
  }

  void writeCommand(uint8_t i2c_converter_command, uint8_t i2c_address, uint8_t register_number,
                    uint8_t value)
  {
    std::vector<uint8_t> buffer;

    uint8_t number_bytes = 1; //static here because we only have one value

    buffer.push_back(i2c_converter_command);
    buffer.push_back(i2c_address);
    buffer.push_back(register_number);
    buffer.push_back(number_bytes);
    buffer.push_back(value);

    serial_->write(buffer);
    serial_->flushOutput();


    //check response (every successful command is going to be answered with non zero
    uint8_t buf = 0;
    //small sleep is required by USB-I2C device, see datasheet
    usleep(5000);
    serial_->read(&buf, 1);
    if (buf == 0x00)
    {
      string err_msg("Connection is broken");
      ROS_ERROR_STREAM(err_msg.c_str());
      throw err_msg;
    }

  }

  void readCommand(uint8_t i2c_converter_command, uint8_t i2c_address, uint8_t register_number,
                      uint8_t* read_buffer, uint8_t number_bytes)
    {
      std::vector<uint8_t> write_buffer;

      write_buffer.push_back(i2c_converter_command);
      //reading is indicated by an odd address, therefore we need to add 1
      write_buffer.push_back(i2c_address + 1);
      write_buffer.push_back(register_number);
      write_buffer.push_back(number_bytes);

      serial_->write(write_buffer);
      serial_->flushOutput();

      //read requested data
      //small sleep is required by USB-I2C device, see datasheet
      usleep(5000);
      serial_->read(read_buffer, number_bytes);

    }

protected:

  boost::shared_ptr<serial::Serial> serial_;

  int i2c_address_;

  uint8_t version_;

};

/**
 * Implementation for the SRF08 sonar sensor with light sensor
 * Documentation http://www.robot-electronics.co.uk/htm/srf08tech.html
 */
class SRF08: public SRFUltrasonsicRangeFinder
{
public:

  SRF08(boost::shared_ptr<serial::Serial> serial_device, int amplification = 0x11, int i2c_address = SRF08_DEFAULT_I2C_ADDRESS) :
    SRFUltrasonsicRangeFinder(serial_device, i2c_address)
  {
    //lower amp => less echoes
    writeCommand(I2C_AD1, i2c_address, SRF08_LIGHT_AND_GAIN_REGISTER, amplification); // amplification of 0x11 works good
    //max range = 6m = 0x8c
    writeCommand(I2C_AD1, i2c_address, SRF_RANGE_REGISTER, 0x8C);
  }

};

/**
 * Implementation for the SRF02 sonar sensor (does not have a light sensor)
 * Documentation http://www.robot-electronics.co.uk/htm/srf02techI2C.htm
 */
class SRF02: public SRFUltrasonsicRangeFinder
{
public:

  SRF02(boost::shared_ptr<serial::Serial> serial_device, int i2c_address = SRF02_DEFAULT_I2C_ADDRESS) :
    SRFUltrasonsicRangeFinder(serial_device, i2c_address)
  {
  }

};

class SRFWithFilter{

public:
  ros::NodeHandle n;

  boost::shared_ptr<SRFUltrasonsicRangeFinder> sensor;
  vector<boost::shared_ptr<IFilter1D>> distance_filters;

  string name_;

  ros::Publisher sonar_pub;

  ros::Publisher light_pub;

  sensor_msgs::Range range_msg;

  sensor_msgs::Illuminance light_msg;

  SRFWithFilter(ros::NodeHandle handle, string name):  n(handle), sensor(NULL), name_(name) {

    if(!name.empty())
    {
      name.append("/");
    }

    string topic_range =name + "sonar";
    string frame_id_range = "sonar_link";

    string topic_light = name + "light";
    string frame_id_light = "light_link";

    sonar_pub = n.advertise<sensor_msgs::Range>(topic_range, 100);

    light_pub = n.advertise<sensor_msgs::Illuminance>(topic_light, 100);

    range_msg.header.frame_id = frame_id_range;
    range_msg.max_range = 6.00;
    range_msg.min_range = 0.03;
    //range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

    light_msg.header.frame_id = frame_id_light;

  }

  void update()
  {
    int light = 0;
    int distance = 0;

    ros::Time now = ros::Time::now();

    bool rc = sensor->requestRanging(UNIT_CM);

    if(rc){
      rc = sensor->updateMeasurement(light, distance);

      if (rc)
      {
        for (std::vector<boost::shared_ptr<IFilter1D>>::iterator it = distance_filters.begin(); it != distance_filters.end(); ++it)
        {
          distance =(*it)->filter(distance);
        }

      ROS_DEBUG_STREAM("Light:" << light << " Distance:" << distance);

      publishRange(distance, now);

      publishLight(light, now);
    }

  }

}

  virtual ~SRFWithFilter(){
  }

private:
  virtual void publishRange(int distance, const ros::Time& now)
  {
    // conversion to meters
    range_msg.range = distance / 100.0;
    range_msg.header.stamp = now;
    range_msg.header.seq++;
    sonar_pub.publish(range_msg);
  }

  virtual void publishLight(int light, const ros::Time& now)
  {
    //TODO
    //conversion to lux
    //bright light 248 maybe 40000lux...
    //darkness light 0
    light_msg.illuminance = light;
    light_msg.header.stamp = now;
    light_msg.header.seq++;
    light_pub.publish(light_msg);
  }
};

}


using namespace srf;

void createAndInitSRFSensor(const ros::NodeHandle& private_n, boost::shared_ptr<serial::Serial> serial_device, boost::shared_ptr<SRFWithFilter>  sensorContainer)
{
  boost::shared_ptr<SRFUltrasonsicRangeFinder> sensor;

  int amplification = 0x11;
  private_n.param("amplification", amplification, amplification);
  bool enable_median = false;
  private_n.param("enable_median_filter", enable_median, enable_median);
  double lowPassSmoothing = 1.0;
  private_n.param("low_pass_smoothing", lowPassSmoothing, lowPassSmoothing);
  ROS_INFO_STREAM("Amp: 0x" << std::hex << amplification);
  string sensor_type;
  private_n.param("sensor_type", sensor_type, string("srf02"));
  ROS_INFO_STREAM("Sensor type: " << sensor_type);
  int i2c_address = 0;
  string i2c_address_str;
  private_n.param("i2c_address", i2c_address_str, i2c_address_str);
  if (!i2c_address_str.empty())
  {
    i2c_address = std::stoul(i2c_address_str, nullptr, 16);
  }
  ROS_INFO_STREAM("I2C Address: 0x" << std::hex << i2c_address);
  int new_i2c_address = 0;
  string new_i2c_address_str;

  private_n.param("change_to_i2c_address", new_i2c_address_str, new_i2c_address_str);
  if (!new_i2c_address_str.empty())
  {
    new_i2c_address = std::stoul(new_i2c_address_str, nullptr, 16);
  }

  if (sensor_type.compare("srf08") == 0)
  {
    if (i2c_address == 0)
    {
      i2c_address = SRF08_DEFAULT_I2C_ADDRESS;
    }
    sensor = boost::shared_ptr<SRFUltrasonsicRangeFinder>(new SRF08(serial_device, amplification, i2c_address));
  }
  else if (sensor_type.compare("srf02") == 0)
  {
    if (i2c_address == 0)
    {
      i2c_address = SRF02_DEFAULT_I2C_ADDRESS;
    }
    sensor = boost::shared_ptr<SRFUltrasonsicRangeFinder>(new SRF02(serial_device, i2c_address));
  }
  else
  {
    ROS_WARN_STREAM("Unknown sensor type " << sensor_type << " using default srf02");
    if (i2c_address == 0)
    {
      i2c_address = SRF02_DEFAULT_I2C_ADDRESS;
    }
    sensor = boost::shared_ptr<SRFUltrasonsicRangeFinder>(new SRF02(serial_device, i2c_address));
  }

  if (new_i2c_address != 0)
  {
    ROS_INFO_STREAM("Trying to change to new i2c address 0x" << std::hex << new_i2c_address);
    sensor->changeI2CAddress(new_i2c_address);
  }

  sensorContainer->sensor = sensor;

  if(enable_median){
    sensorContainer->distance_filters.push_back(boost::shared_ptr<IFilter1D>(new MedianFilterSimple()));
  }
  if(lowPassSmoothing != 1.0){
    sensorContainer->distance_filters.push_back(boost::shared_ptr<IFilter1D>(new LowPassSimple(lowPassSmoothing)));
  }

}

/**
 * The main method is handling initialization as well as the actual update process
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "srf_serial");

  ros::NodeHandle private_n("~");

  try
  {

    /**
     * Init serial connection
     */
    string serial_device_address;
    private_n.param("serial_device", serial_device_address, string("/dev/ttyUSB0"));

    ROS_INFO_STREAM("Using serial port: " << serial_device_address);

    boost::shared_ptr<serial::Serial> serial_device (new serial::Serial(serial_device_address, 19200, serial::Timeout::simpleTimeout(500)));
    if (serial_device->isOpen())
    {
      ROS_DEBUG_STREAM("Serial port is open");
    } else
    {
      ROS_ERROR_STREAM("Serial port can not be opened");
      return -1;
    }

    /**
     * Update rate
     */

    int update_frequency = 40;
    private_n.param("update_frequency", update_frequency, update_frequency);

    ROS_INFO_STREAM("Hertz: " << update_frequency );

    /**
     * Init sensors
     */

    vector<boost::shared_ptr<SRFWithFilter>> sensors;

    //maximum of 16 sensors with parameters in groups starting with name sonar0, sonar1,...
    for(uint16_t i = 0; i < 16; i++){

      string sonar_group("sonar");
      sonar_group.append(std::to_string(i));

      ros::NodeHandle n_sonar(sonar_group);

      //check if group exists by testing name attribute
      if(!n_sonar.hasParam("name")){
        break;
      }else{

        std::string sonar_name;
        n_sonar.param("name", sonar_name, sonar_name);

        ROS_INFO_STREAM("Found sonar: " << sonar_group << " with name: " << sonar_name);

        boost::shared_ptr<SRFWithFilter> sensorContainer(new SRFWithFilter (private_n, sonar_name));

        createAndInitSRFSensor(n_sonar, serial_device, sensorContainer);

        sensors.push_back(sensorContainer);
      }
    }

    //if no groups found check for sonar parameters in node config for single unnamed sonar sensor
    if(sensors.size() == 0){
      //name is optional
      std::string sonar_name;
      private_n.param("name", sonar_name, sonar_name);

      boost::shared_ptr<SRFWithFilter> sensorContainer(new SRFWithFilter (private_n, sonar_name));

      createAndInitSRFSensor(private_n, serial_device, sensorContainer);

      sensors.push_back(sensorContainer);
    }

    /**
     * The actual update loop
     */

    ros::Rate loop_rate(update_frequency);

    while (ros::ok())
    {
      for (std::vector<boost::shared_ptr<SRFWithFilter>>::iterator it = sensors.begin(); it != sensors.end(); ++it)
      {
        //TODO separate into trigger and read
        (*it)->update();
      }

      ros::spinOnce();

      loop_rate.sleep();
    }

  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch (std::exception& e)
  {
	ROS_ERROR("srf08_serial failed %s", e.what());
	return -1;
  }
  catch (...)
  {
	  ROS_ERROR("srf08_serial failed");
    return -1;
  }

  return 0;
}

