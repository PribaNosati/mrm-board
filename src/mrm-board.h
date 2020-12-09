#pragma once

#include "Arduino.h"
#include <BluetoothSerial.h>
#include <mrm-can-bus.h>
#include <mrm-common.h>
#include <mrm-pid.h>
#include <vector>

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
#define COMMAND_FPS_REQUEST 0x30
#define COMMAND_FPS_SENDING 0x31
#define COMMAND_ID_CHANGE_REQUEST 0x40
#define COMMAND_NOTIFICATION 0x41
#define COMMAND_OSCILLATOR_TEST 0x43
#define COMMAND_ERROR 0xEE
#define COMMAND_REPORT_ALIVE 0xFF

#define MAX_MOTORS_IN_GROUP 4

#ifndef toRad
#define toRad(x) ((x) / 180.0 * PI) // Degrees to radians
#endif
#ifndef toDeg
#define toDeg(x) ((x) / PI * 180.0) // Radians to degrees
#endif

enum BoardId{ID_MRM_8x8A, ID_ANY, ID_MRM_BLDC2X50, ID_MRM_BLDC4x2_5, ID_MRM_COL_CAN, ID_MRM_FET_CAN, ID_MRM_IR_FINDER_2, ID_MRM_IR_FINDER3, ID_MRM_IR_FINDER_CAN, 
	ID_MRM_LID_CAN_B, ID_MRM_LID_CAN_B2, ID_MRM_MOT2X50, ID_MRM_MOT4X3_6CAN, ID_MRM_MOT4X10, ID_MRM_NODE, ID_MRM_REF_CAN, ID_MRM_SERVO, ID_MRM_SWITCH, 
	ID_MRM_THERM_B_CAN, ID_MRM_US};

enum BoardType{MOTOR_BOARD, SENSOR_BOARD};

class Robot;

/** Board is a class of all the boards of the same type, not a single board!
*/
class Board{
protected:
	uint32_t _alive; // Responded to ping, maximum 32 devices of the same class, stored bitwise.
	bool _aliveReport = false;
	char _boardsName[12];
	BoardType _boardType; // To differentiate derived boards
	uint8_t canData[8]; // Array used to store temporary CAN Bus data
	uint8_t devicesOnABoard; // Number of devices on a single board
	//std::vector<bool>(maxNumberOfBoards * devicesOn1Board) deviceStarted; //todo - not to allow reading if the device not started.
	uint8_t errorCode = 0;
	uint8_t errorInDeviceNumber = 0;
	std::vector<uint16_t>* fpsLast; // FPS local copy.
	BoardId _id;
	std::vector<uint32_t>* idIn;  // Inbound message id
	std::vector<uint32_t>* idOut; // Outbound message id
	std::vector<uint32_t>* lastMessageReceivedMs;
	std::vector<uint32_t>* _lastReadingMs;
	uint8_t maximumNumberOfBoards;
	uint8_t measuringMode = 0;
	uint8_t measuringModeLimit = 0;
	uint8_t _message[29]; // Message a device sent.
	std::vector<char[10]>* _name;// Device's name
	int nextFree;
	Robot* robotContainer;

	/** Common part of message decoding
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - command found
	*/
	bool messageDecodeCommon(uint32_t canId, uint8_t data[8], uint8_t deviceNumber = 0);

public:
	
	/**
	@param robot - robot containing this board
	@param maxNumberOfBoards - maximum number of boards
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param id - unique id
	*/
	Board(Robot* robot, uint8_t maxNumberOfBoards, uint8_t devicesOnABoard, const char * boardName, BoardType boardType, BoardId id);

	/** Add a device. 
	@param deviceName
	@param canIn
	@param canOut
	*/
	void add(char* deviceName, uint16_t canIn, uint16_t canOut);

	/** Did it respond to last ping? If not, try another ping and see if it responds.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - any alive.
	@param checkAgainIfDead - try another ping
	@param errorIfNotAfterCheckingAgain - the robot will stop. Otherwise only warning displayed.
	@return - alive or not
	*/
	bool alive(uint8_t deviceNumber = 0, bool checkAgainIfDead = false, bool errorIfNotAfterCheckingAgain = false);

	/** Did any device respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	uint8_t count();

	/** Set aliveness
	@param yesOrNo
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void aliveSet(bool yesOrNo, uint8_t deviceNumber = 0);

	BoardType boardType(){ return _boardType; }

	/** Count all the devices, alive or not
	@return - count
	*/
	uint8_t deadOrAliveCount();

	/** Number of devices in each group (board)
	@return - number of devices
	*/
	uint8_t devicesOnASingleBoard() { return this->devicesOnABoard; }

	/** Maximum number of devices in all groups (boards)
	@raturn - number of devices
	*/
	uint8_t devicesMaximumNumberInAllBoards() { return this->devicesOnABoard * this->maximumNumberOfBoards; }

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	@param mask - bitwise, 16 bits - no more than 16 devices! Bit == 1 - scan, 0 - no scan.
	@return - alive count
	*/
	uint8_t devicesScan(bool verbose = true, uint16_t mask = 0xFFFF);

	/** Last error code
	@return - last error code from all devices of this kind
	*/
	uint8_t errorCodeLast() { return errorCode; }

	/** Device which caused last error
	@return - device number
	*/
	uint8_t errorWasInDeviceNumber() { return errorInDeviceNumber; }

	/** Request firmware version
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void firmwareRequest(uint8_t deviceNumber = 0xFF);

	/** Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - FPS
	*/
	uint16_t fps(uint8_t deviceNumber = 0);

	/** Display FPS for all devices
	*/
	void fpsDisplay();

	/** Request Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void fpsRequest(uint8_t deviceNumber = 0xFF);

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
	void info(uint8_t deviceNumber = 0xFF);

	/** Is the frame addressed to this device's Arduino object?
	@param canIdOut - CAN Bus id.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - if true, it is
	*/
	bool isForMe(uint32_t canIdOut, uint8_t deviceNumber);

	/** Does the frame originate from this device's Arduino object?
	@param canIdOut - CAN Bus id.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
	@return - if true, it does
	*/
	bool isFromMe(uint32_t canIdOut, uint8_t deviceNumber);

	/** Last message received
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - milliseconds
	*/
	uint32_t lastMessageMs(uint8_t deviceNumber = 0) { return (*lastMessageReceivedMs)[deviceNumber]; }

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	virtual bool messageDecode(uint32_t canId, uint8_t data[8]) = 0;

	/** Prints a frame
	@param msgId - messageId
	@param dlc - data length
	@param data - payload
	@param outbound - otherwise inbound
	@return -if true, foundand printed
	*/
	bool messagePrint(uint32_t msgId, uint8_t dlc, uint8_t * data, bool outbound);

	/** Send CAN Bus message
	@param dlc - data length
	@param data - payload
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void messageSend(uint8_t* data, uint8_t dlc, uint8_t deviceNumber = 0);

	/** Returns device's name
	@param deviceNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - name
	*/
	char *name(uint8_t deviceNumber);

	/** Returns device group's name
	@return - name
	*/
	char* name() {return _boardsName;}

	/** Request notification
	@param commandRequestingNotification
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void notificationRequest(uint8_t commandRequestingNotification, uint8_t deviceNumber);

	/** Reserved for production
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void oscillatorTest(uint8_t deviceNumber = 0xFF);

	/** Reset
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	*/
	void reset(uint8_t deviceNumber = 0xFF);

	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param measuringModeNow - Measuring mode id. Default 0.
	@param refreshMs - gap between 2 CAN Bus messages to refresh local Arduino copy of device's data. 0 - device's default.
	*/
	void start(uint8_t deviceNumber = 0xFF, uint8_t measuringModeNow = 0, uint16_t refreshMs = 0);

	/** add() assigns device numbers one after another. swap() changes the sequence later. Therefore, add(); add(); will assign number 0 to a device with the smallest CAN Bus id and 1 to the one with the next smallest. 
	If we want to change the order so that now the device 1 is the one with the smalles CAN Bus id, we will call swap(0, 1); after the the add() commands.
	@param deviceNumber1 - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param deviceNumber2 - Second device.
	*/
	void swap(uint8_t deviceNumber1, uint8_t deviceNumber2);

	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void stop(uint8_t deviceNumber = 0xFF);

	/**Test
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param betweenTestsMs - time in ms between 2 tests. 0 - default.
	*/
	virtual void test(uint8_t deviceNumber = 0xFF, uint16_t betweenTestsMs = 0) {}
};



class MotorBoard : public Board {
protected:
	std::vector<uint32_t>* encoderCount; // Encoder count
	std::vector<bool>* reversed; // Change rotation
	std::vector<int8_t>* lastSpeed;
public:

	/**
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param maxNumberOfBoards - maximum number of boards
	@param id - unique id
	*/
	MotorBoard(Robot* robot, uint8_t devicesOnABoard, const char * boardName, uint8_t maxNumberOfBoards, BoardId id);

	/** Changes rotation's direction
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void directionChange(uint8_t deviceNumber);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Encoder readings
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - encoder value
	*/
	uint16_t reading(uint8_t deviceNumber);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Motor speed
	@param motorNumber - motor's number
	@param speed - in range -127 to 127
	*/
	void speedSet(uint8_t motorNumber, int8_t speed);

	/** Stop all motors
	*/
	void stop();

	/**Test
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param betweenTestsMs - time in ms between 2 tests. 0 - default.
	*/
	void test(uint8_t deviceNumber = 0xFF, uint16_t betweenTestsMs = 0);
};




class SensorBoard : public Board {
public:
	/**
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param maxNumberOfBoards - maximum number of boards
	@param id - unique id
	*/
	SensorBoard(Robot* robot, uint8_t devicesOnABoard, const char* boardName, uint8_t maxNumberOfBoards, BoardId id);

	/** Starts periodical CANBus messages that will be refreshing values that mirror sensor's calculated values
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingCalculatedDataStart(uint8_t deviceNumber = 0xFF);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	virtual bool messageDecode(uint32_t canId, uint8_t data[8]){return false;}
};

//typedef void (*SpeedSetFunction)(uint8_t motorNumber, int8_t speed);

class MotorGroup {
protected:
	MotorBoard* motorBoard[MAX_MOTORS_IN_GROUP] = { NULL, NULL, NULL, NULL }; // Motor board for each wheel. It can the same, but need not be.
	uint8_t motorNumber[MAX_MOTORS_IN_GROUP];
	Robot* robotContainer;

	/** Angle between -180 and 180 degrees
	@return - angle
	*/
	float angleNormalized(float angle);
public:
	MotorGroup(Robot* robot);

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
	MotorGroupDifferential(Robot* robot, MotorBoard* motorBoardForLeft1, uint8_t motorNumberForLeft1, MotorBoard* motorBoardForRight1, uint8_t motorNumberForRight1,
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
	MotorGroupStar(Robot* robot, MotorBoard* motorBoardFor45Degrees, uint8_t motorNumberFor45Degrees, MotorBoard* motorBoardFor135Degrees, uint8_t motorNumberFor135Degrees,
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
