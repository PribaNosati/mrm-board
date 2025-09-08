#pragma once

#include <functional>
#include "Arduino.h"
#include "mrm-can-bus.h"
#include "mrm-common.h"
#include "mrm-pid.h"
#include <cstring>
#include <vector>
#include <map>

// Addresses:
// 0x0110 - 272 mrm-bldc2x125
// 0x0150 - 336 mrm-lid-c2, mrm-lid-can-b2
// 0x0160 - 352 mrm-ref-can*
// 0x0170 - 368 mrm-node
// 0x0180 - 384 mrm-lid-c, mrm-lid-can-b
// 0x0200 - 512 mrm-8x8a
// 0x0210 - 528 mrm-therm-b-can
// 0x0230 - 560 mrm-mot4x3.6can
// 0x0240 - 576 mrm-bldc4x2.5
// 0x0250 - 592 mrm-mot4x10
// 0x0260 - 608 mrm-mot2x50
// 0x0270 - 624 mrm-lid-can-b2, mrm-lid-c2
// 0x0280 - 640 mrm-lid-c, mrm-lid-can-b, mrm-therm-l2
// 0x0290 - 656 mrm-ir-finder-can
// 0x0300 - 768 mrm-us
// 0x0310 - 784 mrm-col-can
// 0x0320 - 800 mrm-us-a, mrm-us-u, mrm-us40sg
// 0x0330 - 816 mrm-ir-finder3
// 0x0350 - 848 mrm-fet-can
// 0x0360 - 864 mrm-us-b
// 0x0370 - 880 mrm-us1
// 0x0380 - 896 mrm-col-b
// 0x0390 - 912 mrm-lid-d
// 0x0400 - 1024 mrm-lid-d

// Commands

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_ONCE 0x11
#define COMMAND_SENSORS_MEASURE_STOP 0x12
#define COMMAND_SENSORS_MEASURE_SENDING 0x13
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION 0x14
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA 0x15
#define COMMAND_SENSORS_MEASURE_CALCULATED_SENDING 0x16
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2 0x17
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3 0x18
#define COMMAND_FIRMWARE_REQUEST 0x19
#define COMMAND_FIRMWARE_SENDING 0x1A
#define COMMAND_RESET 0x1B
#define COMMAND_MESSAGE_SENDING_1 0x1C
#define COMMAND_MESSAGE_SENDING_2 0x1D
#define COMMAND_MESSAGE_SENDING_3 0x1E
#define COMMAND_MESSAGE_SENDING_4 0x1F
#define COMMAND_SPEED_SET 0x20
#define COMMAND_SPEED_SET_REQUEST_NOTIFICATION 0x21
#define COMMAND_DUPLICATE_ID_PING 0x22
#define COMMAND_DUPLICATE_ID_ECHO 0x23
#define COMMAND_INFO_REQUEST 0x24
#define COMMAND_INFO_SENDING_1 0x25
#define COMMAND_INFO_SENDING_2 0x26
#define COMMAND_INFO_SENDING_3 0x27
#define COMMAND_PNP_ENABLE 0x28
#define COMMAND_PNP_DISABLE 0x29
#define COMMAND_FPS_REQUEST 0x30
#define COMMAND_FPS_SENDING 0x31
#define COMMAND_PNP_REQUEST 0x32
#define COMMAND_PNP_SENDING 0x33
#define COMMAND_ID_CHANGE_REQUEST 0x40
#define COMMAND_NOTIFICATION 0x41
#define COMMAND_OSCILLATOR_TEST 0x43
#define COMMAND_ERROR 0xEE
#define COMMAND_CAN_TEST 0xFE
#define COMMAND_REPORT_ALIVE 0xFF

#define MRM_MOTORS_INACTIVITY_ALLOWED_MS 10000

#define MAX_MOTORS_IN_GROUP 4
#define PAUSE_MICRO_S_BETWEEN_DEVICE_SCANS 10000

#ifndef toRad
#define toRad(x) ((x) / 180.0 * PI) // Degrees to radians
#endif
#ifndef toDeg
#define toDeg(x) ((x) / PI * 180.0) // Radians to degrees
#endif

class Robot;
class Board;

struct Device{
	public:
	Device(const std::string& name, uint16_t canIdIn, uint16_t canIdOut, uint8_t number)
		: name(name), readingsCount(0), canIdIn(canIdIn), canIdOut(canIdOut), lastMessageReceivedMs(0), lastReadingsMs(0), fpsLast(0xFFFF), number(number), alive(false), aliveOnce(false) {};
	std::string name;
	uint8_t readingsCount;
	uint16_t canIdIn;
	uint16_t canIdOut;
	uint64_t lastMessageReceivedMs;
	uint64_t lastReadingsMs;
	uint16_t fpsLast; //FPS local copy
	uint8_t number;
	bool alive;
	bool aliveOnce;
};

/** Board is a class of all the boards of the same type, not a single board!
*/
class Board{
	public:
	enum BoardId{ID_MRM_8x8A, ID_ANY, ID_MRM_BLDC2X50, ID_MRM_BLDC4x2_5, ID_MRM_COL_B, ID_MRM_COL_CAN, ID_MRM_FET_CAN, ID_MRM_IR_FINDER_2, 
	ID_MRM_IR_FINDER3, ID_MRM_IR_FINDER_CAN, ID_MRM_LID_CAN_B, ID_MRM_LID_CAN_B2, ID_MRM_LID_D, ID_MRM_MOT2X50, ID_MRM_MOT4X3_6CAN, ID_MRM_MOT4X10, 
	ID_MRM_NODE, ID_MRM_REF_CAN, ID_MRM_SERVO, ID_MRM_SWITCH, ID_MRM_THERM_B_CAN, ID_MRM_US, ID_MRM_US_B, ID_MRM_US1};
	enum BoardType{ANY_BOARD, MOTOR_BOARD, SENSOR_BOARD};

protected:
	uint32_t _aliveOnce; // The device was alive at least once after power-on.
	std::string _boardsName;
	BoardType typeId; // To differentiate derived boards
	uint8_t canData[8]; // Array used to store temporary CAN Bus data
	static std::map<int, std::string>* commandNames;
	BoardId _id;
	uint8_t maximumNumberOfBoards;
	uint8_t measuringMode = 0;
	uint8_t measuringModeLimit = 0;
	uint8_t _message[29]; // Message a device sent.
	int nextFree = -1;

	/** Common part of message decoding
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - command found
	*/
	bool messageDecodeCommon(CANMessage& message, Device& device);

public:
	std::vector<Device> devices; // List of devices on this board
	uint8_t devicesOnABoard; // Number of devices on a single board
	uint8_t number; // Index in vector

	// In order to avoid back-pointers to Robot class
	std::function<void (CANMessage& message, uint8_t errorCode, bool peripheral, bool printNow)> errorAddParent;
	std::function<bool ()> userBreakParent;
	std::function<bool ()> setupParent;
	std::function<void()> endParent;
	std::function<void (CANMessage& message, Board* board, uint8_t deviceNumber, bool outbound, bool clientInitiated, std::string postfix)> messagePrintParent;
	std::function<void (CANMessage& message, uint8_t deviceNumber)> messageSendParent;
	std::function<void (uint16_t)> delayMsParent;
	std::function<void ()> noLoopWithoutThisParent;
	std::function<uint16_t (uint16_t timeoutFirst, uint16_t timeoutBetween, bool onlySingleDigitInput, 
		uint16_t limit, bool printWarnings)> serialReadNumberParent;
	
	/**
	@param robot - robot containing this board
	@param maxNumberOfBoards - maximum number of boards
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param id - unique id
	*/
	Board(uint8_t maxNumberOfBoards, uint8_t devicesOnABoard, std::string boardName, BoardType boardType, BoardId id);

	/** Add a device.
	@param deviceName
	@param canIn
	@param canOut
	*/
	void add(std::string deviceName, uint16_t canIn, uint16_t canOut);

	/** Did it respond to last ping? If not, try another ping and see if it responds.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - any alive.
	@param checkAgainIfDead - try another ping
	@param errorIfNotAfterCheckingAgain - the robot will stop. Otherwise only warning displayed.
	@return - alive or not
	*/
	bool aliveWithOptionalScan(Device* device = NULL, bool checkAgainIfDead = false);

	uint8_t aliveCount();

	/** Set aliveness
	@param yesOrNo
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
						0xFF - set all
	*/
	void aliveSet(bool yesOrNo, Device * device = nullptr);

	BoardType boardType(){ return typeId; }

	/** Detects if there is a gap in CAN Bus addresses' sequence, like 0, 2, 3 (missing 1).
	@return - is there a gap.
	*/
	bool canGap();

	virtual std::string commandName(uint8_t byte);

	static std::string commandNameCommon(uint8_t byte);

	/** Did any device respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	uint8_t count();

	void delayMs(uint16_t ms);

	Device* deviceGet(uint8_t deviceNumber);

	uint8_t deviceNumber(uint16_t msgId);

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	@param mask - bitwise, 16 bits - no more than 16 devices! Bit == 1 - scan, 0 - no scan.
	*/
	void devicesScan(uint16_t mask = 0xFFFF);

	void end();
	void errorAdd(CANMessage message, uint8_t errorCode, bool peripheral, bool printNow);

	/** Request firmware version
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void firmwareRequest(Device * device = nullptr);

	/** Display FPS for all devices
	*/
	void fpsDisplay();

	/** Request Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void fpsRequest(Device* device = nullptr);

	/** Board class id, not each device's
	*/
	BoardId id() { return _id; }

	/** Change CAN Bus id
	@param newDeviceNumber - new number
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void idChange(uint16_t newDeviceNumber, uint8_t deviceNumber = 0);
	
	/** Request information
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void info(Device* device = nullptr);

	/** Is the frame addressed to this device's Arduino object?
	@param canIdOut - CAN Bus id.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - if true, it is
	*/
	bool isForMe(uint32_t canId, Device& device);

	/** Does the frame originate from this device's Arduino object?
	@param canIdOut - CAN Bus id.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
	@return - if true, it does
	*/
	bool isFromMe(uint32_t canId, Device& device);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	@return - true if canId for this class
	*/
	virtual bool messageDecode(CANMessage& message)= 0;

	/** Prints a frame
	@param msgId - messageId
	@param dlc - data length
	@param data - payload
	@param outbound - otherwise inbound
	@return -if true, foundand printed
	*/
	void messagePrint(CANMessage& message, bool outbound);

	/** Send CAN Bus message
	@param dlc - data length
	@param data - payload
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void messageSend(uint8_t* data, uint8_t dlc, uint8_t deviceNumber = 0);

	/** Returns device group's name
	@return - name
	*/
	std::string name() {return _boardsName;}

	void noLoopWithoutThis();

	/** Request notification
	@param commandRequestingNotification
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void notificationRequest(uint8_t commandRequestingNotification, Device& device);

	/** Reserved for production
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void oscillatorTest(Device* device = nullptr);

	/** Enable plug and play
	@param enable - enable or disable
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void pnpSet(bool enable = true, Device * device = nullptr);

	/** Reset
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	*/
	void reset(Device* device = nullptr);

	uint16_t serialReadNumber(uint16_t timeoutFirst, uint16_t timeoutBetween, bool onlySingleDigitInput, 
		uint16_t limit, bool printWarnings);

	bool setup();

	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param measuringModeNow - Measuring mode id. Default 0.
	@param refreshMs - gap between 2 CAN Bus messages to refresh local Arduino copy of device's data. 0 - device's default.
	*/
	void start(Device* device = nullptr, uint8_t measuringModeNow = 0, uint16_t refreshMs = 0);

	/** add() assigns device numbers one after another. swap() changes the sequence later. Therefore, add(); add(); will assign number 0 to a device with the smallest CAN Bus id and 1 to the one with the next smallest. 
	If we want to change the order so that now the device 1 is the one with the smalles CAN Bus id, we will call swap(0, 1); after the the add() commands.
	@param deviceNumber1 - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param deviceNumber2 - Second device.
	*/
	void swapCANIds(Device& device1, Device& device2);

	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void stop(Device* device = nullptr);

	/**Test
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param betweenTestsMs - time in ms between 2 tests. 0 - default.
	*/
	virtual void test(Device * device = nullptr, uint16_t betweenTestsMs = 0) {}

	bool userBreak();
};


class MotorBoard : public Board {
protected:
	std::vector<uint32_t>* encoderCount; // Encoder count
	std::vector<bool>* reversed; // Change rotation
	std::vector<int8_t>* lastSpeed;

	/** If sensor not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool started(Device& device);
public:

	/**
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param maxNumberOfBoards - maximum number of boards
	@param id - unique id
	*/
	MotorBoard(uint8_t devicesOnABoard, std::string boardName, uint8_t maxNumberOfBoards, BoardId id);

	~MotorBoard();

	/** Changes rotation's direction
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void directionChange(Device& device);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	@return - true if canId for this class
	*/
	bool messageDecode(CANMessage& message);

	/** Encoder readings
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - encoder value
	*/
	uint16_t reading(Device& device);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Motor speed
	@param motorNumber - motor's number
	@param speed - in range -127 to 127
	*/
	void speedSet(uint8_t motorNumber, int8_t speed, bool force = false);

	/** Stop all motors
	*/
	void stop();

	/**Test
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param betweenTestsMs - time in ms between 2 tests. 0 - default.
	*/
	void test(Device * device , uint16_t betweenTestsMs = 0);
};


class SensorBoard : public Board {
private:
	uint8_t _readingsCount; // Number of measurements, like 9 in a reflectance sensors with 9 transistors

public:
	/**
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param maxNumberOfBoards - maximum number of boards
	@param id - unique id
	*/
	SensorBoard(uint8_t devicesOnABoard, const char* boardName, uint8_t maxNumberOfBoards, BoardId id,
		uint8_t measurementsCount);

	/** Starts periodical CANBus messages that will be refreshing values that mirror sensor's calculated values
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingCalculatedDataStart(Device* device);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	@return - true if canId for this class
	*/
	virtual bool messageDecode(CANMessage& message){return false;}

	/** All readings
	@param subsensorNumberInSensor - like a single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	virtual uint16_t reading(uint8_t subsensorNumberInSensor, uint8_t deviceNumber = 0){ return 0;}

	uint8_t readingsCount(){return _readingsCount;}
};

//typedef void (*SpeedSetFunction)(uint8_t motorNumber, int8_t speed);

class MotorGroup {
protected:
	MotorBoard* motorBoard[MAX_MOTORS_IN_GROUP] = { NULL, NULL, NULL, NULL }; // Motor board for each wheel. It can the same, but need not be.
	uint8_t motorNumber[MAX_MOTORS_IN_GROUP];
public:
	std::function<void (uint16_t)> delayMs;

	MotorGroup();

	/** Stop motors
	*/
	void stop();
};

/** Motor group for tank-like propulsion.
*/
class MotorGroupDifferential : public MotorGroup {
private:
	/** Check if speed is inside bounds
	@param speed - speed to be checked
	@return - speed inside bounds
	*/
	int16_t checkBounds(int16_t speed);

public:
	/** Constructor
	@param motorBoardForLeft1 - Controller for one of the left wheels
	@param motorNumberForLeft1 - Controller's output number
	@param motorBoardForRight1 - Controller for one of the right wheels
	@param motorNumberForRight1 - Controller's output number
	@param motorBoardForLeft2 - Controller for one of the left wheels
	@param motorNumberForLeft2 - Controller's output number
	@param motorBoardForRight2 - Controller for one of the right wheels
	@param motorNumberForRight2 - Controller's output number
	*/
	MotorGroupDifferential(MotorBoard* motorBoardForLeft1, uint8_t motorNumberForLeft1, MotorBoard* motorBoardForRight1, uint8_t motorNumberForRight1,
		MotorBoard* motorBoardForLeft2 = NULL, uint8_t motorNumberForLeft2 = 0, MotorBoard* motorBoardForRight2 = NULL, uint8_t motorNumberForRight2 = 0);

	/** Start all motors
	@param leftSpeed, in range -127 to 127
	@param right Speed, in range -127 to 127
	@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
	*/
	void go(int16_t leftSpeed = 0, int16_t rightSpeed = 0, int16_t lateralSpeedToRight = 0, uint8_t speedLimit = 127);
};

/** Motors' axles for a star - they all point to a central point. Useful for driving soccer robots with omni-wheels.
*/
class MotorGroupStar : public MotorGroup {
public:
	/**
	@param motorBoardFor45Degrees - motor controller for the motor which axle is inclined 45 degrees clockwise from robot's front.
	@param motorNumberFor45Degrees - Controller's output number.
	@param motorBoardFor13Degrees - motor controller for the motor which axle is inclined 135 degrees clockwise from robot's front.
	@param motorNumberFor135Degrees - Controller's output number.
	@param motorBoardForMinus135Degrees - motor controller for the motor which axle is inclined -135 degrees clockwise from robot's front.
	@param motorNumberForMinus135Degrees - Controller's output number.
	@param motorBoardForMinus45Degrees - motor controller for the motor which axle is inclined -45 degrees clockwise from robot's front.
	@param motorNumberForMinus45Degrees - Controller's output number.
	*/
	MotorGroupStar(MotorBoard* motorBoardFor45Degrees, uint8_t motorNumberFor45Degrees, MotorBoard* motorBoardFor135Degrees, uint8_t motorNumberFor135Degrees,
		MotorBoard* motorBoardForMinus135Degrees, uint8_t motorNumberForMinus135Degrees, MotorBoard* motorBoardForMinus45Degrees, uint8_t motorNumberForMinus45Degrees);

	/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
	@param speed - 0 to 100.
	@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
	Values between -180 and 180.
	@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
	numbers because a value 100 turns on all the motors at maximal speed.
	@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
	*/
	void go(float speed, float angleDegrees = 0, float rotation = 0, uint8_t speedLimit = 127);

	/** Moves the robot in order to elinimate errors (for x and y directions).
	@param errorX - X axis error.
	@param errorY - Y axis error.
	@param headingToMaintain - Heading to maintain.
	@param verbose - print details
	*/
	void goToEliminateErrors(float errorX, float errorY, float headingToMaintain, Mrm_pid* pidXY, Mrm_pid* pidRotation, bool verbose = false);
};
