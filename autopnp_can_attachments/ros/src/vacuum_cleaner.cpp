#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <libpcan/libpcan.h>
#include <fcntl.h>
#include <boost/thread/mutex.hpp>

volatile bool running = false;
boost::mutex mutex;
HANDLE device = 0;
std::string devfile;
int baudrate = 0;
int modid = -1;
const int timeout = 1000000;

void close()
{
	if (device)
	{
		CAN_Close(device);
		device = 0;
	}
	running = false;
}
bool receive_and_test(uint16_t id)
{
	TPCANRdMsg rd;

	if (!device)
		return false;

	bool test = false;

	while (ros::ok())
	{
		int iRet = LINUX_CAN_Read_Timeout(device, &rd, timeout);
		//CAN_Status(device);

		//ROS_INFO_STREAM("ret: "<< iRet);

		if (iRet == CAN_ERR_OK)
		{
			//ROS_INFO_STREAM("id: "<< rd.Msg.ID << " vs. " << id);
			if (rd.Msg.ID == id)
				test = true;
		}
		else if (iRet & CAN_ERR_QRCVEMPTY)
		{
			break; // no more messages
		}
		else
		{
			close();
			break;
		}
	}

	return test;
}

bool transmit(uint16_t id, int len, uint8_t b0 = 0, uint8_t b1 = 0, uint8_t b2 = 0, uint8_t b3 = 0, uint8_t b4 = 0, uint8_t b5 = 0, uint8_t b6 = 0, uint8_t b7 = 0)
{
	if (!device)
		return false;
	TPCANMsg m;
	// copy CMsg to TPCmsg
	m.LEN = len >= 0 ? len : 0;
	m.ID = id;
	m.MSGTYPE = len >= 0 ? MSGTYPE_STANDARD : MSGTYPE_RTR;

	m.DATA[0] = b0;
	m.DATA[1] = b1;
	m.DATA[2] = b2;
	m.DATA[3] = b3;
	m.DATA[4] = b4;
	m.DATA[5] = b5;
	m.DATA[6] = b6;
	m.DATA[7] = b7;

	if (LINUX_CAN_Write_Timeout(device, &m, timeout) != CAN_ERR_OK)
	{
		close();
		return false;
	}

	ros::Duration(0.1).sleep();

	int ret = CAN_Status(device);
	//ROS_INFO_STREAM("ret " << ret);

	return (ret==CAN_ERR_OK || ret==32);
}

bool check_nmt()
{
	bool ok = false;
	if (device)
	{
		transmit(0x700 + modid, -1);
		ok = receive_and_test(0x700 + modid);
	}
	if (!ok)
		close();
	return ok;
}

void nmt_timer(const ros::TimerEvent&)
{
	boost::mutex::scoped_lock lock(mutex);
	check_nmt();
}

bool power_on(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	boost::mutex::scoped_lock lock(mutex);
	response.success = running && device && transmit(0x200 + modid, 8, 0xff, 0x7F); // without pwm
	//response.success.data = running && device && transmit(0x200 + modid,8,0,80); //pwm: 80*256 / 0x8000 (why?)
	return true;
}
bool power_off(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	boost::mutex::scoped_lock lock(mutex);
	response.success = running && device && transmit(0x200 + modid, 8, 0, 0);
	return true;
}

bool shutdown(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	boost::mutex::scoped_lock lock(mutex);
	close();
	response.success = true;
	return true;
}

bool init(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	boost::mutex::scoped_lock lock(mutex);

	response.success = false;

	if (running)
	{
		response.success = true;
		return true;
	}

	running = false;
	if (device)
		CAN_Close(device);

	device = LINUX_CAN_Open(devfile.c_str(), O_RDWR);

	if (device)
	{
		CAN_ResetFilter(device);
		WORD btr0btr1 = LINUX_CAN_BTR0BTR1(device, baudrate);
		if (CAN_Init(device, btr0btr1, CAN_INIT_TYPE_ST) == CAN_ERR_OK)
		{
			CAN_Status(device);	// clear/reset error
			if (transmit(0, 2, 0x81, 0) && transmit(0, 2, 0x01, 0))
			{
				running = check_nmt();
			}
			else
			{
				response.message = "Could not transmit";
			}
		}
		else
		{
			response.message = "Could not initialize";
		}
	}
	else
	{
		response.message = "Could not open";
	}

	if (!running && device)
	{
		ROS_ERROR("Closing device");
		CAN_Close(device);
		device = 0;
	}

	response.success = running;

	return true;
}

bool recover(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	boost::mutex::scoped_lock lock(mutex);

	response.success = false;

	if (running && device)
	{
		response.success = transmit(0, 2, 0x81, 0) && transmit(0, 2, 0x01, 0);
	}
	return true;
}

void diag(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (running && device)
	{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "vacuum cleaner initialized and running");
	}
	else
	{
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "vacuum cleaner not initialized");
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "autopnp_can_attachments");

	ros::MultiThreadedSpinner spinner;
	ros::NodeHandle nh;

	std::cout << "---------- Vacuum cleaner parameters ----------\n";
	nh.param("CanDevice", devfile, std::string("/dev/pcanusb1"));
	std::cout << "CanDevice=" << devfile << std::endl;
	nh.param("CanBaudrate", baudrate, 500000);
	std::cout << "CanBaudrate=" << baudrate << std::endl;
	nh.param("ModId", modid, 13);
	std::cout << "ModId=" << modid << std::endl;

	ros::ServiceServer init_srv = nh.advertiseService("init", init);
	ros::ServiceServer recover_srv = nh.advertiseService("recover", recover);
	ros::ServiceServer shutdown_srv = nh.advertiseService("shutdown", shutdown);
	ros::ServiceServer power_on_srv = nh.advertiseService("power_on", power_on);
	ros::ServiceServer power_off_srv = nh.advertiseService("power_off", power_off);

	diagnostic_updater::Updater updater;
	updater.setHardwareID("none");

	updater.add("status", diag);

	ros::Timer updater_timer = nh.createTimer(ros::Duration(updater.getPeriod() / 2.0), boost::bind(&diagnostic_updater::Updater::update, &updater));
	ros::Timer alive = nh.createTimer(ros::Duration(1), nmt_timer);

	spinner.spin();

	if (device)
	{
		CAN_Close(device);
	}
	return 0;
}
