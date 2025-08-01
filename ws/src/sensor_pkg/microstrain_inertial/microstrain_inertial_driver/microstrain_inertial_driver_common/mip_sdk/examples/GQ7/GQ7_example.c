
/////////////////////////////////////////////////////////////////////////////
//
// GQ7_Example.c
//
// C Example set-up program for the GQ7
//
// This example shows a typical setup for the GQ7 sensor in a wheeled-vehicle application using C.
// It is not an exhaustive example of all GQ7 settings.
// If your specific setup needs are not met by this example, please consult
// the MSCL-embedded API documentation for the proper commands.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include "example_utils.h"

#include <mip/mip_all.h>
#include <microstrain/connections/serial/serial_port.h>
#include <stdio.h>
#include <stdlib.h>


////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

int port = -1;
mip_interface device;

//Sensor-to-vehicle frame transformation (Euler Angles)
float sensor_to_vehicle_transformation_euler[3] = {0.0, 0.0, 0.0};

//GNSS antenna offsets
float gnss1_antenna_offset_meters[3] = {-0.25, 0.0, 0.0};
float gnss2_antenna_offset_meters[3] = {0.25, 0.0, 0.0};

//Device data stores
mip_shared_gps_timestamp_data sensor_gps_time;
mip_sensor_scaled_accel_data  sensor_accel;
mip_sensor_scaled_gyro_data   sensor_gyro;
mip_sensor_scaled_mag_data    sensor_mag;

mip_gnss_fix_info_data        gnss_fix_info[2];

bool gnss_fix_info_valid[2] = {false};

mip_shared_gps_timestamp_data filter_gps_time;
mip_filter_status_data        filter_status;
mip_filter_position_llh_data  filter_position_llh;
mip_filter_velocity_ned_data  filter_velocity_ned;
mip_filter_euler_angles_data  filter_euler_angles;

bool filter_state_full_nav = false;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);
bool should_exit();


////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{

    //
    //Process arguments
    //

    if(argc != 3)
        return usage(argv[0]);

    const char* port_name = argv[1];
    uint32_t baudrate     = atoi(argv[2]);

    if(baudrate == 0)
        return usage(argv[0]);

    mip_example_init();

    printf("Connecting to and configuring sensor.\n");

    //
    //Open the device port
    //

    if(!serial_port_open(&device_port, port_name, baudrate))
        exit_gracefully("ERROR: Could not open device port!");


    //
    //Initialize the MIP interface
    //

    mip_interface_init(
        &device, mip_timeout_from_baudrate(baudrate), 1000,
        &mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL
    );



    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(mip_base_ping(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(mip_base_set_idle(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(mip_3dm_default_device_settings(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    //
    //Setup Sensor data format to 100 Hz
    //

    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet (shown in GNSS setup).

    if(mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    const mip_descriptor_rate sensor_descriptors[4] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,     sensor_decimation },
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,  sensor_decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,   sensor_decimation },
    };

    if(mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 4, sensor_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup GNSS 1 and 2 data format to 2 Hz (decimation of 1)
    //

    const mip_descriptor_rate gnss_descriptors[1] = {
        { MIP_DATA_DESC_GNSS_FIX_INFO, 1 }
     };

    //GNSS1
    if(mip_3dm_write_message_format(&device, MIP_GNSS1_DATA_DESC_SET, 1, gnss_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS1 message format!");

    //GNSS2
    if(mip_3dm_write_message_format(&device, MIP_GNSS2_DATA_DESC_SET, 1, gnss_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS2 message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    const mip_descriptor_rate filter_descriptors[5] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_POS_LLH,          filter_decimation },
        { MIP_DATA_DESC_FILTER_VEL_NED,          filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
    };

    if(mip_3dm_write_message_format(&device, MIP_FILTER_DATA_DESC_SET, 5, filter_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    //Setup the sensor to vehicle transformation
    //

    if(mip_3dm_write_sensor_2_vehicle_transform_euler(&device, sensor_to_vehicle_transformation_euler[0], sensor_to_vehicle_transformation_euler[1], sensor_to_vehicle_transformation_euler[2]) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation!");

    //
    //Setup the GNSS antenna offsets
    //

    //GNSS1
    if(mip_filter_write_multi_antenna_offset(&device, 1, gnss1_antenna_offset_meters) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS1 antenna offset!");

    //GNSS2
    if(mip_filter_write_multi_antenna_offset(&device, 2, gnss2_antenna_offset_meters) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS2 antenna offset!");


    //
    //Setup the filter aiding measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    //

    if(mip_filter_write_aiding_measurement_enable(&device, MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable!");

    if(mip_filter_write_aiding_measurement_enable(&device, MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_HEADING, true) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable!");


    //
    //Enable the wheeled-vehicle constraint
    //

    if(mip_filter_write_wheeled_vehicle_constraint_control(&device, 1) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter wheeled-vehicle constraint enable!");


    //
    //Setup the filter initialization (note: heading alignment is a bitfield!)
    //

    float filter_init_pos[3] = {0};
    float filter_init_vel[3] = {0};

    if(mip_filter_write_initialization_configuration(&device, 0, MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT,
       MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA | MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC,
       0.0, 0.0, 0.0, filter_init_pos, filter_init_vel, MIP_FILTER_REFERENCE_FRAME_LLH) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter initialization configuration!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(mip_filter_reset(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Sensor Data
    mip_dispatch_handler sensor_data_handlers[4];

    mip_interface_register_extractor(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SHARED_GPS_TIME,     extract_mip_shared_gps_timestamp_data_from_field, &sensor_gps_time);
    mip_interface_register_extractor(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED, extract_mip_sensor_scaled_accel_data_from_field,  &sensor_accel);
    mip_interface_register_extractor(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED,  extract_mip_sensor_scaled_gyro_data_from_field,   &sensor_gyro);
    mip_interface_register_extractor(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED,   extract_mip_sensor_scaled_mag_data_from_field,    &sensor_mag);

    //GNSS Data
    mip_dispatch_handler gnss_data_handlers[2];

    mip_interface_register_extractor(&device, &gnss_data_handlers[0], MIP_GNSS1_DATA_DESC_SET, MIP_DATA_DESC_GNSS_FIX_INFO, extract_mip_gnss_fix_info_data_from_field, &gnss_fix_info[0]);
    mip_interface_register_extractor(&device, &gnss_data_handlers[1], MIP_GNSS2_DATA_DESC_SET, MIP_DATA_DESC_GNSS_FIX_INFO, extract_mip_gnss_fix_info_data_from_field, &gnss_fix_info[1]);

    //Filter Data
    mip_dispatch_handler filter_data_handlers[5];

    mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_SHARED_GPS_TIME,         extract_mip_shared_gps_timestamp_data_from_field, &filter_gps_time);
    mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,        &filter_status);
    mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_POS_LLH,          extract_mip_filter_position_llh_data_from_field,  &filter_position_llh);
    mip_interface_register_extractor(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_VEL_NED,          extract_mip_filter_velocity_ned_data_from_field,  &filter_velocity_ned);
    mip_interface_register_extractor(&device, &filter_data_handlers[4], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);

    //
    //Resume the device
    //

    if(mip_base_resume(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Main Loop: Update the interface and process data
    //

    bool          running              = true;
    mip_timestamp prev_print_timestamp = 0;

    printf("Sensor is configured... waiting for filter to enter Full Navigation mode.\n");

    char *state_init = "";
    char **current_state = &state_init;
    while(running)
    {
        mip_interface_update(&device, 0, false);
        displayFilterState(filter_status.filter_state, current_state, false);

        //Check GNSS fixes and alert the user when they become valid
        for(int i=0; i<2; i++)
        {
            if((gnss_fix_info_valid[i] == false) && ((gnss_fix_info[i].fix_type == MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_3D) ||
                                                     (gnss_fix_info[i].fix_type == MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FLOAT) ||
                                                     (gnss_fix_info[i].fix_type == MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FIXED)) &&
                (gnss_fix_info[i].valid_flags & MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_TYPE))
            {
                printf("NOTE: GNSS%i fix info valid.\n", i+1);
                gnss_fix_info_valid[i] = true;
            }
        }

        //Check Filter State
        if((!filter_state_full_nav) && (filter_status.filter_state == MIP_FILTER_MODE_FULL_NAV))
        {
            printf("NOTE: Filter has entered full navigation mode.\n");
            filter_state_full_nav = true;
        }

        //Once in full nav, print out data at 1 Hz
        if(filter_state_full_nav)
        {
           mip_timestamp curr_time = get_current_timestamp();

           if(curr_time - prev_print_timestamp >= 1000)
           {
                printf("TOW = %f: POS_LLH = [%f, %f, %f], VEL_NED = [%f, %f, %f], ATT_EULER = [%f %f %f]\n",
                       filter_gps_time.tow, filter_position_llh.latitude, filter_position_llh.longitude, filter_position_llh.ellipsoid_height,
                       filter_velocity_ned.north, filter_velocity_ned.east, filter_velocity_ned.down,
                       filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);

                prev_print_timestamp = curr_time;
           }
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");
}

////////////////////////////////////////////////////////////////////////////////
// Print Usage Function
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0)
{
    printf("Usage: %s <port> <baudrate>\n", argv0);
    return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
  return false;

}
