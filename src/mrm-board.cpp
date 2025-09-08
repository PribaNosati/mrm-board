#include "mrm-board.h"
#include <mrm-pid.h>
#include "mrm-robot.h"

#define REPORT_STRAY 1
#define REQUEST_NOTIFICATION 0

std::map<int, std::string>* Board::commandNames = NULL;

/** Board is a single instance for all boards of the same type, not a single board (if there are more than 1 of the same type)! */

/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param maxNumberOfBoards - maximum number of boards
@param devicesOn1Board - number of devices on each board
@param boardName - board's name
@param id - unique id
*/
Board::Board(uint8_t maxNumberOfBoards, uint8_t devicesOn1Board, std::string boardName, BoardType boardType, BoardId id) {
	this->devicesOnABoard = devicesOn1Board;
	this->maximumNumberOfBoards = maxNumberOfBoards;
	this->_boardsName = boardName;
	nextFree = 0;
	typeId = boardType;
	_message[28] = '\0';
	_id = id;

	if (Board::commandNames == NULL)
	{
		Board::commandNames = new std::map<int, std::string>();
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_CONTINUOUS, "Measure cont"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_ONCE, "Measure once"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_STOP, "Measure stop"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_SENDING, "Measure send"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, "Meas req not"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA, "Meas con cal"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_CALCULATED_SENDING, "Meas cal sen"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2, "Measure co 2"});
		Board::commandNames->insert({COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3, "Measure co 3"});
		Board::commandNames->insert({COMMAND_FIRMWARE_REQUEST, "Firmware req"});
		Board::commandNames->insert({COMMAND_FIRMWARE_SENDING, "Firmware sen"});
		Board::commandNames->insert({COMMAND_RESET, "Reset"});
		Board::commandNames->insert({COMMAND_MESSAGE_SENDING_1, "Messa send 1"});
		Board::commandNames->insert({COMMAND_MESSAGE_SENDING_2, "Messa send 2"});
		Board::commandNames->insert({COMMAND_MESSAGE_SENDING_3, "Messa send 3"});
		Board::commandNames->insert({COMMAND_MESSAGE_SENDING_4, "Messa send 4"});
		Board::commandNames->insert({COMMAND_SPEED_SET, "Speed set   "});
		Board::commandNames->insert({COMMAND_SPEED_SET_REQUEST_NOTIFICATION, "Speed set re"});
		Board::commandNames->insert({COMMAND_DUPLICATE_ID_PING, "Dupl id ping"});
		Board::commandNames->insert({COMMAND_DUPLICATE_ID_ECHO, "Dupl id echo"});
		Board::commandNames->insert({COMMAND_INFO_REQUEST, "Info request"});
		Board::commandNames->insert({COMMAND_INFO_SENDING_1, "Info sendi 1"});
		Board::commandNames->insert({COMMAND_INFO_SENDING_2, "Info sendi 2"});
		Board::commandNames->insert({COMMAND_INFO_SENDING_3, "Info sendi 3"});
		Board::commandNames->insert({COMMAND_FPS_REQUEST, "FPS request "});
		Board::commandNames->insert({COMMAND_FPS_SENDING, "FPS sending "});
		Board::commandNames->insert({COMMAND_ID_CHANGE_REQUEST, "Id change re"});
		Board::commandNames->insert({COMMAND_NOTIFICATION, "Notification"});
		Board::commandNames->insert({COMMAND_OSCILLATOR_TEST, "Oscilla test"});
		Board::commandNames->insert({COMMAND_ERROR, "Error"});
		Board::commandNames->insert({COMMAND_CAN_TEST, "CAN test"});
		Board::commandNames->insert({COMMAND_REPORT_ALIVE,  "Report alive"});
	}
}

/** Add a device.
@param deviceName
@param canIn
@param canOut
*/
void Board::add(std::string deviceName, uint16_t canIn, uint16_t canOut) {
if (deviceName.length() > 9) {
		sprintf(errorMessage, "Name too long: %s", deviceName.c_str());
		return;
	}
	devices.push_back({deviceName, canIn, canOut, (uint8_t)devices.size()});
	nextFree++;
}

/** Did it respond to last ping? If not, try another ping and see if it responds.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - any alive.
@param checkAgainIfDead - try another ping
@param errorIfNotAfterCheckingAgain - the robot will stop. Otherwise only warning displayed.
@return - alive or not
*/
bool Board::aliveWithOptionalScan(Device* device, bool checkAgainIfDead) {
	if (device == NULL) {
		for (Device& dev : devices)
			if (aliveWithOptionalScan(&dev, checkAgainIfDead))
				return true;
		return false;
	}
	else {
		if (device->alive)
			return true;
		else if (checkAgainIfDead) {
			devicesScan();
			if (device->alive)
				return true;
			else {
				sprintf(errorMessage, "%s dead", device->name.c_str());
				return false;
			}
		}
		else
			return false;
	}
}

uint8_t Board::aliveCount(){
	uint8_t count = 0;
	for (Device& device : devices) 
		if (device.alive)
			count++;
	return count;
}


/** Set aliveness
@param yesOrNo
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
					0xFF - set all
*/
void Board::aliveSet(bool yesOrNo, Device * device) {
	if (device == nullptr) {
		for (Device& dev : devices)
			aliveSet(yesOrNo, &dev);
	}
	else{
		device->alive = yesOrNo;
		if (yesOrNo)
			device->aliveOnce = true;
	}
}


/** Detects if there is a gap in CAN Bus addresses' sequence, like 0, 2, 3 (missing 1).
@return - is there a gap.
*/
bool Board::canGap() {
	bool dead = false;
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (aliveWithOptionalScan(&devices[deviceNumber])){
			if (dead)
				return true;
		}
		else
			dead = true;

	}
	return false;
}

std::string Board::commandName(uint8_t byte){
	return "";
}

std::string Board::commandNameCommon(uint8_t byte){
	auto it = commandNames->find(byte);
	if (it == commandNames->end())
		return "Warning: no common command found for key " + (int)byte;
	else
		return it->second;
}


/** Did any device respond to last ping?
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
uint8_t Board::count() {
	uint8_t cnt = 0;
	for(Device& device : devices)
		if (aliveWithOptionalScan(&device))
			cnt++;
	return cnt;
}


void Board::delayMs(uint16_t ms){
	if (delayMsParent)
		delayMsParent(ms);
	else{
		print("delayMsParent() not defined.\n\r");
		exit(1);
	}
}


Device* Board::deviceGet(uint8_t deviceNumber){
	if (deviceNumber < devices.size())
		return &devices[deviceNumber];
	else
		return nullptr;
}


uint8_t Board::deviceNumber(uint16_t msgId){
	for(auto device: devices)
		if (isForMe(msgId, device) || isFromMe(msgId, device)) 
			return device.number;
	return 0xFF;
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
@param mask - bitwise, 16 bits - no more than 16 devices! Bit == 1 - scan, 0 - no scan.
*/
void Board::devicesScan(uint16_t mask) {
	for (Device& device: devices) {
		if (((mask >> device.number) & 1) && !device.alive) { // If in the list requested to be scanned.
			delayMs(5);
			canData[0] = COMMAND_REPORT_ALIVE;
			messageSend(canData, 1, device.number);
			delayMicroseconds(id() == BoardId::ID_MRM_8x8A ? PAUSE_MICRO_S_BETWEEN_DEVICE_SCANS * 3 :  PAUSE_MICRO_S_BETWEEN_DEVICE_SCANS); // Exchange CAN Bus messages and receive possible answer, that sets _alive.
		}
	}
}


void Board::end(){
	if (endParent)
		endParent();
	else{
		print("endParent() not defined.\n\r");
		exit(1);
	}
}


void Board::errorAdd(CANMessage message, uint8_t errorCode, bool peripheral, bool printNow){
	if (errorAddParent)
		errorAddParent(message, errorCode, peripheral, printNow);
	else{
		print("errorAddParent() not defined.\n\r");
		exit(1);
	}
}


/** Request firmware version
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::firmwareRequest(Device * device) {
	if (device == nullptr) {
		for (Device& dev : devices)
			firmwareRequest(&dev);
	}
	else {
		if (device->alive) {
			canData[0] = COMMAND_FIRMWARE_REQUEST;
			messageSend(canData, 1, device->number);
		}
	}
}


/** Display FPS for all devices
*/
void Board::fpsDisplay() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (aliveWithOptionalScan(&devices[deviceNumber])){
			if (devices[deviceNumber].fpsLast == 0xFFFF)
				print("%s: no response\n\r", devices[deviceNumber].name.c_str());
			else
				print("%s: %i FPS\n\r", devices[deviceNumber].name.c_str(), devices[deviceNumber].fpsLast);
		}
	}
}

/** Request Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.  0xFF - for all devices.
*/
void Board::fpsRequest(Device* device) {
	if (device == nullptr) {
		for (Device& dev : devices)
			fpsRequest(&dev);
	}
	else {
		if (device->alive) {
			canData[0] = COMMAND_FPS_REQUEST;
			messageSend(canData, 1, device->number);
			device->fpsLast = 0xFFFF;
		}
	}
}

/** Change CAN Bus id
@param newId - CAN Bus id
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::idChange(uint16_t newDeviceNumber, uint8_t deviceNumber) {
	canData[0] = COMMAND_ID_CHANGE_REQUEST;
	canData[1] = newDeviceNumber;
	messageSend(canData, 2, deviceNumber);
}


/** Request information
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
*/
void Board::info(Device* device) {
	if (device == nullptr) {
		for (Device& dev : devices)
			info(&dev);
	}
	else {
		if (device->alive) {
			canData[0] = COMMAND_INFO_REQUEST;
			messageSend(canData, 1, device->number);
			delayMs(1);
		}
	}
}


/** Is the frame addressed to this device's Arduino object?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it is
*/
bool Board::isForMe(uint32_t canId, Device& device) {

	return canId == device.canIdOut;
}

/** Does the frame originate from this device's Arduino object?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it does
*/
bool Board::isFromMe(uint32_t canId, Device& device) {

	return canId == device.canIdIn;
}

/** Common part of message decoding
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - command found
*/
bool Board::messageDecodeCommon(CANMessage& message, Device& device) {
	device.lastMessageReceivedMs = millis();
	bool found = true;
	uint8_t command = message.data[0];
	switch (command) {
	case COMMAND_DUPLICATE_ID_ECHO:
	case COMMAND_DUPLICATE_ID_PING:
		break;
	case COMMAND_ERROR:
		errorAdd(message, message.data[1], true, true);
		break;
	case COMMAND_FIRMWARE_SENDING: {
		uint16_t firmwareVersion = (message.data[2] << 8) | message.data[1];
		print("%s: ver. %i \n\r", device.name.c_str(), firmwareVersion);
	}
		break;
	case COMMAND_FPS_SENDING:
		device.fpsLast = (message.data[2] << 8) | message.data[1];
		break;
	case COMMAND_MESSAGE_SENDING_1:
		for (uint8_t i = 0; i < 7; i++)
			_message[i] = message.data[i + 1];
		break;
	case COMMAND_MESSAGE_SENDING_2:
		for (uint8_t i = 0; i < 7; i++)
			_message[7 + i] = message.data[i + 1];
		break;
	case COMMAND_MESSAGE_SENDING_3:
		for (uint8_t i = 0; i < 7; i++)
			_message[14 + i] = message.data[i + 1];
		break;
	case COMMAND_MESSAGE_SENDING_4:
		for (uint8_t i = 0; i < 7; i++)
			_message[21 + i] = message.data[i + 1];
		print("Message from %s: %s\n\r", device.name.c_str(), _message);
		break;
	case COMMAND_NOTIFICATION:
	case COMMAND_CAN_TEST:
		break;
	case COMMAND_REPORT_ALIVE:
		device.alive = true;
		break;
	default:
		found = false;
	}
	return found;
}


/** Prints a frame
@param msgId - messageId
@param dlc - data length
@param data - payload
@param outbound - otherwise inbound
@return - if true, found and printed
*/
void Board::messagePrint(CANMessage& message, bool outbound) {
	messagePrintParent(message, this, 0xFF, outbound, false, "");
}

/** Send CAN Bus message
@param dlc - data length
@param data - payload
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::messageSend(uint8_t* data, uint8_t dlc, uint8_t deviceNumber) {
	if (messageSendParent){
		CANMessage message(devices[deviceNumber].canIdIn, data, dlc);
		messageSendParent(message, deviceNumber);
	}
	else{
		print("messageSendParent() not defined.\n\r");
		exit(1);
	}
}


void Board::noLoopWithoutThis(){
	if (noLoopWithoutThisParent)
		noLoopWithoutThisParent();
	else{
		print("noLoopWithoutThisParent() not defined.\n\r");
		exit(1);
	}
}

/** Request notification
@param commandRequestingNotification
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::notificationRequest(uint8_t commandRequestingNotification,  Device& device) {
	printf("THIS FUNCTION DOESN'T WORK\n\r");
	//while (1);
	//uint8_t tries = 0;
	//while (tries < 10) {
	//	tries++;
	//	canData[0] = commandRequestingNotification;
	//	messageSend(canData, 1, deviceNumber);
	//	while (tries != 0xFF && !dequeEmpty()) {
	//		noLoopWithoutThis();
	//		uint32_t id = dequeBack()->messageId;
	//		//print("RCVD id 0x%x, data: 0x%x\n\r", id, mrm_can_bus->dequeBack->data[0]);
	//		if (isForMe(id, deviceNumber) && dequeBack()->data[0] == COMMAND_NOTIFICATION) {
	//			tries = 0xFF;
	//			//print("OK...\n\r");
	//		}
	//	}
	//}
	//if (tries != 0xFF)
	//	strcpy(errorMessage, "Notification failed");
}


/** Reserved for production
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::oscillatorTest(Device* device) {
	if (device == nullptr) {
		for (Device& dev: devices)
			oscillatorTest(&dev);
	}
	else {
		if (device->alive) {
			print("Test %s\n\r", device->name.c_str());
			canData[0] = COMMAND_OSCILLATOR_TEST;
			messageSend(canData, 1, device->number);
		}
	}
}

/** Enable plug and play
@param enable - enable or disable
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::pnpSet(bool enable, Device * device){
	if (device == nullptr)
		for (Device& dev : devices)
			pnpSet(enable, &dev);
	else if (device->alive) {
		delayMs(1);
		canData[0] = enable ? COMMAND_PNP_ENABLE : COMMAND_PNP_DISABLE;
		canData[1] = enable;
		messageSend(canData, 2, device->number);
		print("%s PnP %s\n\r", device->name.c_str(), enable ? "on" : "off");
	}
}


/** Reset
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
*/
void Board::reset(Device* device) {
	if (device == nullptr)
		for (Device& dev: devices)
			reset(&dev);
	else {
		canData[0] = COMMAND_RESET;
		messageSend(canData, 1, device->number);
	}
}


uint16_t Board::serialReadNumber(uint16_t timeoutFirst, uint16_t timeoutBetween, bool onlySingleDigitInput, 
		uint16_t limit, bool printWarnings){
	if (serialReadNumberParent)
		return serialReadNumberParent(timeoutFirst, timeoutBetween, onlySingleDigitInput, 
			limit, printWarnings);
	else{
		print("serialReadNumberParent() not defined.\n\r");
		exit(1);
	}
}


bool Board::setup(){
	if (setupParent)
		return setupParent();
	else{
		print("setupParent() not defined.\n\r");
		exit(1);
	}
}

/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param measuringModeNow - Measuring mode id. Default 0.
@param refreshMs - gap between 2 CAN Bus messages to refresh local Arduino copy of device's data. 0 - device's default.
*/
void Board::start(Device* device, uint8_t measuringModeNow, uint16_t refreshMs) {
	if (device == nullptr)
		for (Device& dev: devices)
			start(&dev, measuringModeNow, refreshMs);
	else {
		if (device->alive) {
			// print("Alive, start reading: %s\n\r", _boardsName.c_str());
#if REQUEST_NOTIFICATION
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, device);
#else
			if (measuringModeNow == 0 || measuringModeLimit == 0)
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
			else if (measuringModeNow == 1 || measuringModeLimit >= 1)
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2;
			else
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3;
			measuringMode = measuringModeNow;
			if (refreshMs != 0) {
				canData[1] = refreshMs & 0xFF;
				canData[2] = (refreshMs >> 8) & 0xFF;
			}
			messageSend(canData, refreshMs == 0 ? 1 : 3, device->number);

			// if (++dumpCnt >= DUMP_LIMIT)
			// 	dumpCnt = 0;
			// dumpMs[dumpCnt] = millis() - dumpLastMs;
			// dumpLastMs = millis();

			delayMs(1); // Otherwise only 4 devices of the same kind started.
#endif
		}
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::stop(Device * device) {
	if (device == nullptr)
		for (Device& dev : devices)
			stop(&dev);
	else {
		if (device->alive) {
			
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			messageSend(canData, 1, device->number);
			device->lastReadingsMs = 0;
			delayMs(1); // TODO
		}
	}
}


/** add() assigns device numbers one after another. swap() changes the sequence later. Therefore, add(); add(); will assign number 0 to a device with the smallest CAN Bus id and 1 to the one with the next smallest.
If we want to change the order so that now the device 1 is the one with the smalles CAN Bus id, we will call swap(0, 1); after the the add() commands.
@param deviceNumber1 - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param deviceNumber2 - Second device.
*/
void Board::swapCANIds(Device& device1, Device& device2) {
	uint16_t idInTemp = device1.canIdIn;
	uint16_t idOutTemp = device1.canIdOut;
	device1.canIdIn = device2.canIdIn;
	device1.canIdOut = device2.canIdOut;
	device2.canIdIn = idInTemp;
	device2.canIdOut = idOutTemp;
}


bool Board::userBreak(){
	if (userBreakParent)
		return userBreakParent();
	else{
		print("userBreakParent() not defined.\n\r");
		exit(1);
	}
}

/**
@param robot - robot containing this board
@param devicesOnABoard - number of devices on each board
@param boardName - board's name
@param maxNumberOfBoards - maximum number of boards
@param id - unique id
*/
MotorBoard::MotorBoard(uint8_t devicesOnABoard, std::string boardName, uint8_t maxNumberOfBoards, BoardId id) :
	Board(maxNumberOfBoards, devicesOnABoard, boardName, MOTOR_BOARD, id) {
	encoderCount = new std::vector<uint32_t>(devicesOnABoard * maxNumberOfBoards);
	reversed = new std::vector<bool>(devicesOnABoard * maxNumberOfBoards);
	lastSpeed = new std::vector<int8_t>(devicesOnABoard * maxNumberOfBoards);
}

MotorBoard::~MotorBoard(){
	stop();
}

/** Changes rotation's direction
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void MotorBoard::directionChange(Device& device) {
	(*reversed)[device.number] = !(*reversed)[device.number];
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool MotorBoard::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint32_t enc = (message.data[4] << 24) | (message.data[3] << 16) | (message.data[2] << 8) | message.data[1];
					(*encoderCount)[device.number] = enc;
					device.lastReadingsMs = millis();
					break;
				}
				default:
					errorAddParent(message, ERROR_COMMAND_UNKNOWN, false, true);
				}
			}
			return true;
		}
	return false;
}


/** Encoder readings
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - encoder value
*/
uint16_t MotorBoard::reading(Device& device) {
	aliveWithOptionalScan(&device, true);
	if (started(device))
		return (*encoderCount)[device.number];
	else
		return 0;
}

/** Print all readings in a line
*/
void MotorBoard::readingsPrint() {
	print("Encoders:");
	for (Device& device : devices)
		if (device.alive)
			print(" %4i", (*encoderCount)[device.number]);
}


/** Motor speed
@param motorNumber - motor's number
@param speed - in range -127 to 127
*/
void MotorBoard::speedSet(uint8_t motorNumber, int8_t speed, bool force) {
	if (motorNumber >= devices.size()) {
		sprintf(errorMessage, "Mot. %i doesn't exist", motorNumber);
		return;
	}

	if (!force && (*lastSpeed)[motorNumber] == speed)
		return;
	(*lastSpeed)[motorNumber] = speed;

	if ((*reversed)[motorNumber])
		speed = -speed;

	canData[0] = COMMAND_SPEED_SET;
	canData[1] = speed + 128;
	messageSend(canData, 2, motorNumber);
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool MotorBoard::started(Device& device) {
	if (millis() - device.lastReadingsMs > MRM_MOTORS_INACTIVITY_ALLOWED_MS || device.lastReadingsMs == 0) {
		// print("Start mrm-bldc4x2.5%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&device, 0);
			// Wait for 1. message.
			uint64_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - device.lastReadingsMs < 100) {
					// print("BLDC confirmed\n\r");
					return true;
				}
				delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), device.number);
		return false;
	}
	else
		return true;
}

/** Stop all motors
*/
void MotorBoard::stop() {
	for (Device& dev : devices) {
			speedSet(dev.number, 0, true);
			delayMs(2);
			canData[0] = COMMAND_SENSORS_MEASURE_STOP; // Stop encoders
			messageSend(canData, 1, dev.number);
			delayMs(3);
		}
}

/**Test
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param betweenTestsMs - time in ms between 2 tests. 0 - default.
*/
void MotorBoard::test(Device * device , uint16_t betweenTestsMs)
{
	const uint16_t PAUSE_MS = 20;
	const uint16_t DISPLAY_PAUSE_MS = 300;
	// const uint8_t STEP = 1;
	const uint8_t MAXIMUM_SPEED = 100; // Max. 127

	const int8_t startSpeed[3] = { 0, MAXIMUM_SPEED, -MAXIMUM_SPEED };
	const int8_t endSpeed[3] = { MAXIMUM_SPEED, -MAXIMUM_SPEED, 0 };
	const int8_t step[3] = { 1, -1, 1 };

	// Select motor
	print("%s - enter motor number [0-%i] or wait for all\n\r", name().c_str(), devices.size() - 1);
	uint16_t selectedMotor = serialReadNumber(3000, 500, devices.size() - 1 > 9, devices.size() - 1, false);
	if (selectedMotor == 0xFFFF)
		print("Test all\n\r");
	else
		print("\n\rTest motor %i\n\r", selectedMotor);

	// Select speed
	bool fixedSpeed = false;
	print("Enter speed [0-127] or wait for all\n\r");
	uint16_t selectedSpeed = serialReadNumber(2000, 500, false, 127, false);
	if (selectedSpeed == 0xFFFF) {
		fixedSpeed = false;
		print("All speeds\n\r");
	}
	else {
		fixedSpeed = true;
		print("\n\rSpeed %i\n\r", selectedSpeed);
	}

	bool goOn = true;
	bool encodersStarted[4] = { false, false, false, false };
	uint32_t lastMs = 0;
	while (goOn) {
		for (Device& dev : devices) {

			if ((selectedMotor != 0xFFFF && dev.number != selectedMotor) || !dev.alive)
				continue;

			if (!encodersStarted[dev.number]) {
				encodersStarted[dev.number] = true;
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
				messageSend(canData, 1, dev.number);
			}

			if (fixedSpeed) {
				speedSet(dev.number, selectedSpeed);
				if (userBreak())
					goOn = false;
				delayMs(PAUSE_MS);
				continue;
			}

			for (uint8_t group = 0; group < 3 && goOn; group++)
				for (int16_t speed = startSpeed[group];
				((step[group] > 0 && speed <= endSpeed[group]) || (step[group] < 0 && speed >= endSpeed[group])) && goOn;
					speed += step[group]) {
				//blink();
				if (userBreak())
					goOn = false;

				speedSet(dev.number, speed);

				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("Mot. %i:%3i, en: %i\n\r", dev.number, speed, (*encoderCount)[dev.number]);
					lastMs = millis();
				}
				delayMs(PAUSE_MS);
			}

			delayMs(2);
			speedSet(dev.number, 0);
		}
	}

	// Stop all motors
	for (Device& dev : devices) {
		speedSet(dev.number, 0);

		if (encodersStarted[dev.number]) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			messageSend(canData, 1, dev.number);
		}
	}

	stop();
}




/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param devicesOnABoard - number of devices on each board
@param boardName - board's name
@param maxNumberOfBoards - maximum number of boards
@param id - unique id
*/
SensorBoard::SensorBoard(uint8_t devicesOnABoard, const char boardName[], uint8_t maxNumberOfBoards, 
	BoardId id, uint8_t readingsCount) :
	Board(maxNumberOfBoards, devicesOnABoard, boardName, SENSOR_BOARD, id) {
		_readingsCount = readingsCount;
}

/** Starts periodical CANBus messages that will be refreshing values that mirror sensor's calculated values
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::continuousReadingCalculatedDataStart(Device* device) {
	if (device == nullptr)
		for (Device& dev : devices)
			continuousReadingCalculatedDataStart(&dev);
	else {
		if (device->alive) {
			// print("Alive, start reading: %s\n\r", name(deviceNumber));
#if REQUEST_NOTIFICATION // todo
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, device);
#else
			canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA;
			messageSend(canData, 1, device->number);
			//print("Sent to 0x%x\n\r, ", (*idIn)[deviceNumber]);
#endif
		}
	}
}


MotorGroup::MotorGroup(){
}


/** Stop motors
*/
void MotorGroup::stop() {
	for (uint8_t i = 0; i < MAX_MOTORS_IN_GROUP; i++)
		if (motorBoard[i] == NULL)
			break;
		else 
			motorBoard[i]->speedSet(motorNumber[i], 0);
}

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
MotorGroupDifferential::MotorGroupDifferential(MotorBoard* motorBoardForLeft1, uint8_t motorNumberForLeft1, MotorBoard* motorBoardForRight1, uint8_t motorNumberForRight1,
	MotorBoard* motorBoardForLeft2, uint8_t motorNumberForLeft2, MotorBoard * motorBoardForRight2, uint8_t motorNumberForRight2) : MotorGroup() {
	motorBoard[0] = motorBoardForLeft1;
	motorNumber[0] = motorNumberForLeft1;
	motorBoard[1] = motorBoardForRight1;
	motorNumber[1] = motorNumberForRight1;
	motorBoard[2] = motorBoardForLeft2;
	motorNumber[2] = motorNumberForLeft2;
	motorBoard[3] = motorBoardForRight2;
	motorNumber[3] = motorNumberForRight2;
}

/** Check if speed is inside bounds
@param speed - speed to be checked
@return - speed inside bounds
*/
int16_t MotorGroupDifferential::checkBounds(int16_t speed) {
	if (speed < -127)
		return -127;
	else if (speed > 127)
		return 127;
	else
		return speed;
}

/** Start all motors
@param leftSpeed, in range -127 to 127
@param right Speed, in range -127 to 127
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void MotorGroupDifferential::go(int16_t leftSpeed, int16_t rightSpeed, int16_t lateralSpeedToRight, uint8_t speedLimit) {
	for (uint8_t i = 0; i < 4; i++)
		if (motorBoard[i] == NULL)
			return;

	if (speedLimit == 0)
		stop();
	else {
		if (speedLimit > 127)
			speedLimit = 127;
		int16_t speeds[4];
		speeds[0] = checkBounds(leftSpeed - lateralSpeedToRight);
		speeds[1] = checkBounds(leftSpeed + lateralSpeedToRight);
		speeds[2] = checkBounds(-rightSpeed + lateralSpeedToRight);
		speeds[3] = checkBounds(-rightSpeed - lateralSpeedToRight);
		float maxSpeed = abs(speeds[0]);
		for (int i = 1; i < 4; i++)
			if (abs(speeds[i]) > maxSpeed)
				maxSpeed = abs(speeds[i]);
		// print("M0:%i M1:%i M2:%i M3:%i Lat:%i\n\r", speeds[0], speeds[1], speeds[2], speeds[3], lateralSpeedToRight);
		for (uint8_t i = 0; i < 4; i++) {
						delayMs(1);
			if (maxSpeed > speedLimit) {
				motorBoard[i]->speedSet(motorNumber[i], (int8_t)(speeds[i] / maxSpeed * speedLimit));
			}
			else {
				motorBoard[i]->speedSet(motorNumber[i], (int8_t)speeds[i]);
			}
		}
	}
}

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
MotorGroupStar::MotorGroupStar(MotorBoard* motorBoardFor45Degrees, uint8_t motorNumberFor45Degrees, MotorBoard* motorBoardFor135Degrees, uint8_t motorNumberFor135Degrees,
	MotorBoard* motorBoardForMinus135Degrees, uint8_t motorNumberForMinus135Degrees, MotorBoard* motorBoardForMinus45Degrees, uint8_t motorNumberForMinus45Degrees){
	motorBoard[0] = motorBoardFor45Degrees;
	motorNumber[0] = motorNumberFor45Degrees;
	motorBoard[1] = motorBoardFor135Degrees;
	motorNumber[1] = motorNumberFor135Degrees;
	motorBoard[2] = motorBoardForMinus135Degrees;
	motorNumber[2] = motorNumberForMinus135Degrees;
	motorBoard[3] = motorBoardForMinus45Degrees;
	motorNumber[3] = motorNumberForMinus45Degrees;
}

/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
@param speed - 0 to 100.
@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
Values between -180 and 180.
@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
numbers because a value 100 turns on all the motors at maximal speed.
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void MotorGroupStar::go(float speed, float angleDegrees, float rotation, uint8_t speedLimit) {
	if (motorBoard[0] != NULL) {
		if (speedLimit == 0)
			stop();
		else {
			angleDegrees += 135;// -= 45;
			float angleRadians = angleDegrees / 180 * 3.14;
			float si = sin(angleRadians);
			float co = cos(angleRadians);
			float xMinus135Deg = -speed * si + rotation; //135 degrees
			float x135Deg = -speed * co + rotation; //45 degrees
			float x45Deg = speed * si + rotation;  //-135 degrees
			float xMinus45Deg = speed * co + rotation;  //-45 degrees
			float speeds[4] = { x45Deg, x135Deg,xMinus135Deg,xMinus45Deg };

			if (speedLimit > 127)
				speedLimit = 127;
			float maxSpeed = abs(speeds[0]);
			for (int i = 1; i < 4; i++)
				if (abs(speeds[i]) > maxSpeed)
					maxSpeed = abs(speeds[i]);

			//Serial.print("Rot err: " + (String)rotation + " ");
			for (int i = 0; i < 4; i++){
				if (maxSpeed > speedLimit) {
					motorBoard[i]->speedSet(motorNumber[i], (int8_t)(speeds[i] / maxSpeed * speedLimit));
					//Serial.print("MAX ");
				}
				else {
					motorBoard[i]->speedSet(motorNumber[i], (int8_t)speeds[i]);
					//Serial.print((String)speeds[i] + " ");
				}
				delayMs(1);
			}
			//Serial.println();
		}
	}
}

/** Moves the robot in order to elinimate errors (for x and y directions).
@param errorX - X axis error.
@param errorY - Y axis error.
@param headingToMaintain - Heading to maintain.
@param verbose - print details
*/
void MotorGroupStar::goToEliminateErrors(float errorX, float errorY, float errorRotation, Mrm_pid *pidXY, Mrm_pid *pidRotation, bool verbose) {
	static uint32_t lastMs = 0;
	// Direction to maximally decrease error
	float heading;
	if (fabsf(errorX) > 0.001) // To avoid overflow.
		heading = toDeg(atan2f(errorX, errorY));
	else
		heading = errorY > 0 ? 0 : -180;
	//Serial.print(" Heading: " + (String)(int)heading + " ");

	// Speed to decrease direction error
	float speed;//todo - correct x - y directions according to heading error.
	errorRotation = angleNormalized(errorRotation);
	if (abs(errorRotation) < 20) // If a rotational error is quite big, correct without moving to x or y directions.
		speed = pidXY->calculate(fabsf(errorX) + fabsf(errorY), false); // PID sets the speed.
	else
		speed = 0;

	// Rotation to decrease z-axis rotation error
	float rotation = pidRotation->calculate(errorRotation);
	//Serial.println("\n\rABS: " + (String)(fabsf(errorX) + fabsf(errorY)) + "Speed " + (String)(int)speed + ", Rot: " + (String)(int)rotation);

	if (false && millis() - lastMs > 500) {
		lastMs = millis();
		//print(" Errors: (%i, %i) %i deg. Sp: %i, rot %i. \n\r", errorX, errorY, heading, speed, rotationToMaintainHeading(headingToMaintain));
	}
#define SPEED_LIMIT 30
	go(speed, heading, rotation, SPEED_LIMIT);
	if (verbose)
		Serial.println("Sp: " + (String)(int)speed + ", head: " + (String)(int)heading + ", rot: " + (int)rotation);
}
