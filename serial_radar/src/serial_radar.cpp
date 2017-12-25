#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>


uint8_t sensor_configuration[8];
uint8_t sensor_back[8];
uint8_t sensor_status[8];
uint8_t target_status[8];
uint8_t target_info[8];
uint8_t buffer[14];

//define starting and ending bytes
#define HDRBYTES      0xAAAA
#define EDRBYTES      0x5555

//define Message ID of 5 different messages
#define MSGID_SC      0x0002
#define MSGID_SB      0x0004
#define MSGID_SS      0x060A
#define MSGID_TS      0x070B
#define MSGID_TI      0x070C

//define the buffer
#define HEADER        *(uint16_t *)(buffer)
#define MESSAGE_ID    *(uint16_t *)(buffer + 2)
#define MESSAGE_INFO  *(uint64_t *)(buffer + 4)
#define ENDER         *(uint16_t *)(buffer + 12)

//define sensor_status

//define target status
//#define NO_NoOfTarget *(uint8_t *)(target_status)
//#define NO_RollCount  *(uint8_t *)(target_status + 1)

//define target info
#define TI_Index      *(uint8_t *)(target_info)
#define TI_Rcs        *(uint8_t *)(target_info + 1)
#define TI_RangeH     *(uint8_t *)(target_info + 2)
#define TI_RangeL     *(uint8_t *)(target_info + 3)
#define TI_Azimuth    *(uint8_t *)(target_info + 4)
#define TI_VreIH      *(uint8_t *)(target_info + 5)
#define TI_VreIL      *(uint8_t *)(target_info + 6)
#define TI_SNR        *(uint8_t *)(target_info + 7)



using namespace std;

int main(int argc, char *argv[])
{
	//creat ros handler to node
	ros::init(argc, argv, "serial_radar");
	ros::NodeHandle serial_radar_nh("~");

	serial::Serial fd;

	string serialPort;

	if(serial_radar_nh.getParam("serialPort", serialPort))
		printf("Retrived Port Name: %s\n", serialPort.data());
	else
	{
		printf("Cannot retrived Port name. Exit\n");
		exit(-1);
	}

	fd.setPort(serialPort.data());
	fd.setBaudrate(115200);
	fd.setTimeout(5, 10, 2, 10, 2);
	fd.open();
	if (fd.isOpen())
	{
		fd.flushInput();
		printf("Connection established\n\n");
	}
	else
	{
		printf("serialInit: Failed to open port\n");
		return 0;
	}

	ros::Rate rate(60);

	while(ros::ok())
	{
		uint32_t bytes_in_buff = fd.available();
		uint32_t read_bytes = 0;

		if (bytes_in_buff > 0)
		{
			while(read_bytes < bytes_in_buff)
			{
				uint8_t temp_byte;
				fd.read(&temp_byte, 1);

				for (int i = 0; i < 13; i++)
					buffer[i] = buffer[i+1];
				
				buffer[13] = temp_byte;
				read_bytes++;
//				printf("0x%x\n", temp_byte);

				if (HEADER == HDRBYTES && ENDER == EDRBYTES)
				{
//					printf("Message ID : 0x%x\n", MESSAGE_ID);

//					if(MESSAGE_ID == MSGID_SS)
//					{
//
//					}

					if(MESSAGE_ID == MSGID_TS)
					{
						
						ros::Time listeningtime = ros::Time::now();
						printf("\n\n\n\n\n\n\n");
						cout<<"Message received time: "<<listeningtime<<endl;
						printf("Target status:");
						printf("            No of targets:   %x\n", buffer[4]);
						printf("                          Rollcount:     %x\n\n", buffer[5]);	
							
					}
						

					else if(MESSAGE_ID == MSGID_TI)
					{
						for(int k = 0; k < 8; k++)
							target_info[k] = buffer[k+4];

						printf("Target %x info:", TI_Index);
						double Rcs = TI_Rcs*0.5 - 50;
						double Range = (TI_RangeH*256 + TI_RangeL)*0.01;
						double Azimuth = TI_Azimuth*2 - 90;
						double Vrel = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
						double SNR = TI_SNR - 127;
//						printf("            Reflected area:      %3.2f\n", Rcs);
						printf("\n                          Distance:            %3.2f\n", Range);
						printf("                          Angle:               %3.2f\n", Azimuth);
						printf("                          Relative speed:      %3.2f\n", Vrel);
//						printf("                          Signal noise ratio:  %3.2f\n\n", SNR);

					}

				}

			}
		}

		rate.sleep();

	}

}


