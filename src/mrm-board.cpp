#include "mrm-board.h"
#include <mrm-pid.h>
#include "mrm-robot.h"

#define REPORT_STRAY 1
#define REQUEST_NOTIFICATION 0

std::vector<uint8_t>* commandIndexes =  new std::vector<uint8_t>(); // C++ 17 enables static variables without global initialization, but no C++ 17 here
std::vector<String>* commandNames =  new std::vector<String>();

/** Board is a single instance for all boards of the same type, not a single board (if there are more than 1 of the same type)! */

/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param maxNumberOfBoards - maximum number of boards
@param devicesOn1Board - number of devices on each board
@param boardName - board's name
@param id - unique id
*/
Board::Board(Robot* robot, uint8_t maxNumberOfBoards, uint8_t devicesOn1Board, const char boardName[], BoardType boardType, BoardId id) {
	robotContainer = robot;
	idIn = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	idOut = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	_name = new std::vector<char[10]>(maxNumberOfBoards * devicesOn1Board);
	fpsLast = new std::vector<uint16_t>(maxNumberOfBoards * devicesOn1Board);
	lastMessageReceivedMs = new std::vector<uint64_t>(maxNumberOfBoards * devicesOn1Board);
	_lastReadingMs = new std::vector<uint64_t>(maxNumberOfBoards * devicesOn1Board);
	this->devicesOnABoard = devicesOn1Board;
	this->maximumNumberOfBoards = maxNumberOfBoards;
	strcpy(this->_boardsName, boardName);
	_alive = 0;
	nextFree = 0;
	_boardType = boardType;
	_message[28] = '\0';
	_id = id;

	for (uint8_t deviceNumber = 0; deviceNumber < maxNumberOfBoards; deviceNumber++)
		(*_lastReadingMs)[deviceNumber] = 0;

	if (commandIndexes == NULL){
		commandIndexes = new std::vector<uint8_t>();
		commandNames = new std::vector<String>();
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_CONTINUOUS);
		commandNames->push_back("Measure cont");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_ONCE);
		commandNames->push_back("Measure once");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_STOP);
		commandNames->push_back("Measure stop");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_SENDING);
		commandNames->push_back("Measure send");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION);
		commandNames->push_back("Meas req not");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA);
		commandNames->push_back("Meas con cal");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_CALCULATED_SENDING);
		commandNames->push_back("Meas cal sen");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2);
		commandNames->push_back("Measure co 2");
		commandIndexes->push_back((uint8_t)COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3);
		commandNames->push_back("Measure co 3");
		commandIndexes->push_back((uint8_t)COMMAND_FIRMWARE_REQUEST);
		commandNames->push_back("Firmware req");
		commandIndexes->push_back((uint8_t)COMMAND_FIRMWARE_SENDING);
		commandNames->push_back("Firmware sen");
		commandIndexes->push_back((uint8_t)COMMAND_RESET);
		commandNames->push_back("Reset");
		commandIndexes->push_back((uint8_t)COMMAND_MESSAGE_SENDING_1);
		commandNames->push_back("Messa send 1");
		commandIndexes->push_back((uint8_t)COMMAND_MESSAGE_SENDING_2);
		commandNames->push_back("Messa send 2");
		commandIndexes->push_back((uint8_t)COMMAND_MESSAGE_SENDING_3);
		commandNames->push_back("Messa send 3");
		commandIndexes->push_back((uint8_t)COMMAND_MESSAGE_SENDING_4);
		commandNames->push_back("Messa send 4");
		commandIndexes->push_back((uint8_t)COMMAND_SPEED_SET);
		commandNames->push_back("Speed set");
		commandIndexes->push_back((uint8_t)COMMAND_SPEED_SET_REQUEST_NOTIFICATION);
		commandNames->push_back("Speed set re");
		commandIndexes->push_back((uint8_t)COMMAND_DUPLICATE_ID_PING);
		commandNames->push_back("Dupl id ping");
		commandIndexes->push_back((uint8_t)COMMAND_DUPLICATE_ID_ECHO);
		commandNames->push_back("Dupl id echo");
		commandIndexes->push_back((uint8_t)COMMAND_INFO_REQUEST);
		commandNames->push_back("Info request");
		commandIndexes->push_back((uint8_t)COMMAND_INFO_SENDING_1);
		commandNames->push_back("Info sendi 1");
		commandIndexes->push_back((uint8_t)COMMAND_INFO_SENDING_2);
		commandNames->push_back("Info sendi 2");
		commandIndexes->push_back((uint8_t)COMMAND_INFO_SENDING_3);
		commandNames->push_back("Info sendi 3");
		commandIndexes->push_back((uint8_t)COMMAND_FPS_REQUEST);
		commandNames->push_back("FPS request");
		commandIndexes->push_back((uint8_t)COMMAND_FPS_SENDING);
		commandNames->push_back("FPS sending");
		commandIndexes->push_back((uint8_t)COMMAND_ID_CHANGE_REQUEST);
		commandNames->push_back("Id change re");
		commandIndexes->push_back((uint8_t)COMMAND_NOTIFICATION);
		commandNames->push_back("Notification");
		commandIndexes->push_back((uint8_t)COMMAND_OSCILLATOR_TEST);
		commandNames->push_back("Oscilla test");
		commandIndexes->push_back((uint8_t)COMMAND_ERROR);
		commandNames->push_back("Error");
		commandIndexes->push_back((uint8_t)COMMAND_REPORT_ALIVE);
		commandNames->push_back("Report alive");
	}
}

/** Add a device.
@param deviceName
@param canIn
@param canOut
*/
void Board::add(char* deviceName, uint16_t canIn, uint16_t canOut) {
	if (nextFree >= devicesMaximumNumberInAllBoards()) {
		sprintf(errorMessage, "Too many %s: %i.", _boardsName, nextFree);
		return;
	}
	if (deviceName != 0) {
		if (strlen(deviceName) > 9) {
			sprintf(errorMessage, "Name too long: %s", deviceName);
			return;
		}
		strcpy((*_name)[nextFree], deviceName);
	}
	(*idIn)[nextFree] = canIn;
	(*idOut)[nextFree] = canOut;
	(*lastMessageReceivedMs)[nextFree] = 0;
	(*fpsLast)[nextFree] = 0xFFFF;
	nextFree++;
}

/** Did it respond to last ping? If not, try another ping and see if it responds.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - any alive.
@param checkAgainIfDead - try another ping
@param errorIfNotAfterCheckingAgain - the robot will stop. Otherwise only warning displayed.
@return - alive or not
*/
bool Board::alive(uint8_t deviceNumber, bool checkAgainIfDead, bool errorIfNotAfterCheckingAgain) {
	if (deviceNumber == 0xFF) {
		for (uint8_t i = 0; i < nextFree; i++)
			if (alive(i, checkAgainIfDead, errorIfNotAfterCheckingAgain))
				return true;
		return false;
	}
	else {
		if (deviceNumber > 31) {
			sprintf(errorMessage, "Device number for %s too big: %i.", _boardsName, deviceNumber);
			return false;
		}
		if ((_alive >> deviceNumber) & 1)
			return true;
		else if (checkAgainIfDead) {
			devicesScan(false);
			if ((_alive >> deviceNumber) & 1)
				return true;
			else {
				if (errorIfNotAfterCheckingAgain)
					sprintf(errorMessage, "%s no. %i dead", name(), deviceNumber);
				else
					print("%s %i dead\n\r", name(), deviceNumber);
				return false;
			}
		}
		else
			return false;
	}
}

uint8_t Board::aliveCount(){
	uint8_t count = 0;
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (aliveGet(deviceNumber))
			count++;
	return count;
}

/** Get aliveness
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return alive or not
*/
bool Board::aliveGet(uint8_t deviceNumber){
	return (_alive >> deviceNumber) & 1;
}

/** Get aliveness at least once after power-on
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return alive or not
*/
bool Board::aliveOnceGet(uint8_t deviceNumber){
	return (_aliveOnce >> deviceNumber) & 1;
}



/** Set aliveness
@param yesOrNo
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
					0xFF - set all
*/
void Board::aliveSet(bool yesOrNo, uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		_alive = yesOrNo ? 0xFF : 0x00;
	else{
		if (deviceNumber > 31) {
			sprintf(errorMessage, "%s: device number too big: %i", name(), deviceNumber);
			return;
		}
		_alive = (_alive & ~(1 << deviceNumber)) | (yesOrNo << deviceNumber);
		if (yesOrNo)
			_aliveOnce = _alive | (1 << deviceNumber);
	}
}


/** Detects if there is a gap in CAN Bus addresses' sequence, like 0, 2, 3 (missing 1).
@return - is there a gap.
*/
bool Board::canGap() {
	bool dead = false;
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (alive(deviceNumber)){
			if (dead)
				return true;
		}
		else
			dead = true;

	}
	return false;
}

String Board::commandName(uint8_t byte){
	return "";
}

void Board::commandNamePrint(uint8_t commandIndex){
	for (int j = 0; j < commandIndexes->size(); j++)
		if (commandIndexes->at(j) == commandIndex){
			print(" command: %s", commandNames->at(j));
			break;
		}
}

/** Did any device respond to last ping?
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
uint8_t Board::count() {
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < nextFree; i++)
		if (alive(i))
			cnt++;
	return cnt;
}

/** Count all the devices, alive or not
@return - count
*/
uint8_t Board::deadOrAliveCount() { return nextFree; }

/** Ping devices and refresh alive array
@param verbose - prints statuses
@param mask - bitwise, 16 bits - no more than 16 devices! Bit == 1 - scan, 0 - no scan.
*/
void Board::devicesScan(bool verbose, uint16_t mask) {
	_aliveReport = verbose; // If true, Board::messageDecodeCommon() will print board's name
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (((mask >> deviceNumber) & 1) && !aliveGet(deviceNumber)) { // If in the list requested to be scanned.
			canData[0] = COMMAND_REPORT_ALIVE;
			messageSend(canData, 1, deviceNumber);
			robotContainer->delayMicros(500); // Exchange CAN Bus messages and receive possible answer, that sets _alive. 
		}
	}
	//print("%s OVER\n\r", nameGroup);
	// return count();
}

/** Request firmware version
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::firmwareRequest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			firmwareRequest(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_FIRMWARE_REQUEST;
			messageSend(canData, 1, deviceNumber);
		}
	}
}

/** Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
@return - FPS
*/
uint16_t Board::fps(uint8_t deviceNumber) {
	return (*fpsLast)[deviceNumber];
}

/** Display FPS for all devices
*/
void Board::fpsDisplay() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (alive(deviceNumber)){
			if ((*fpsLast)[deviceNumber] == 0xFFFF)
				print("%s: no response\n\r", (*_name)[deviceNumber]);
			else
				print("%s: %i FPS\n\r", (*_name)[deviceNumber], fps(deviceNumber));
		}
	}
}

/** Request Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.  0xFF - for all devices.
*/
void Board::fpsRequest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			fpsRequest(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_FPS_REQUEST;
			messageSend(canData, 1, deviceNumber);
			(*fpsLast)[deviceNumber] = 0xFFFF;
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
void Board::info(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			info(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_INFO_REQUEST;
			messageSend(canData, 1, deviceNumber);
			delay(1);
		}
	}
}


/** Is the frame addressed to this device's Arduino object?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it is
*/
bool Board::isForMe(uint32_t canIdOut, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		sprintf(errorMessage, "%s: board doesn't exist: %i", name(), deviceNumber);
		return false;
	}
	return canIdOut == (*idOut)[deviceNumber];
}

/** Does the frame originate from this device's Arduino object?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it does
*/
bool Board::isFromMe(uint32_t canIdOut, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		sprintf(errorMessage, "%s: next free too big: %i", name(), deviceNumber);
		return false;
	}
	return canIdOut == (*idIn)[deviceNumber];
}

/** Common part of message decoding
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - command found
*/
bool Board::messageDecodeCommon(uint32_t canId, uint8_t data[8], uint8_t deviceNumber) {
	(*lastMessageReceivedMs)[deviceNumber] = millis();
	bool found = true;
	uint8_t command = data[0];
	switch (command) {
	case COMMAND_DUPLICATE_ID_ECHO:
	case COMMAND_DUPLICATE_ID_PING:
		break;
	case COMMAND_ERROR:
		errorCode = data[1];
		errorInDeviceNumber = deviceNumber;
		print("Error %i in %s.\n\r", errorCode, (*_name)[deviceNumber]);
		break;
	case COMMAND_FIRMWARE_SENDING: {
		uint16_t firmwareVersion = (data[2] << 8) | data[1];
		print("%s: ver. %i \n\r", (*_name)[deviceNumber], firmwareVersion);
	}
		break;
	case COMMAND_FPS_SENDING:
		(*fpsLast)[deviceNumber] = (data[2] << 8) | data[1];
		break;
	case COMMAND_MESSAGE_SENDING_1:
		for (uint8_t i = 0; i < 7; i++)
			_message[i] = data[i + 1];
		break;
	case COMMAND_MESSAGE_SENDING_2:
		for (uint8_t i = 0; i < 7; i++)
			_message[7 + i] = data[i + 1];
		break;
	case COMMAND_MESSAGE_SENDING_3:
		for (uint8_t i = 0; i < 7; i++)
			_message[14 + i] = data[i + 1];
		break;
	case COMMAND_MESSAGE_SENDING_4:
		for (uint8_t i = 0; i < 7; i++)
			_message[21 + i] = data[i + 1];
		print("Message from %s: %s\n\r", (*_name)[deviceNumber], _message);
		break;
	case COMMAND_NOTIFICATION:
		break;
	case COMMAND_REPORT_ALIVE:
		if (_aliveReport)
			print("%s alive.\n\r", name(deviceNumber));
		aliveSet(true, deviceNumber);
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
bool Board::messagePrint(uint32_t msgId, uint8_t dlc, uint8_t* data, bool outbound) {
	bool found = false;
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(msgId, deviceNumber) || isFromMe(msgId, deviceNumber)) {
			print("%s id:%s (0x%02X)", outbound ? "To" : "From", (*_name)[deviceNumber], msgId);
			for (uint8_t i = 0; i < dlc; i++) {
				if (i == 0)
					commandNamePrint(data[0]);
				else
					print(" %02X", data[i]);
			}
			print("\n\r");
			found = true;
		}
	if (!found) {
		print("%s id:0x%02X", outbound ? "Out" : "In", msgId);
		for (uint8_t i = 0; i < dlc; i++) {
			if (i == 0)
				commandNamePrint(data[0]);
			else
				print(" %02X", data[i]);
		}
		print("\n\r");
	}
	return found;
}

/** Send CAN Bus message
@param dlc - data length
@param data - payload
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::messageSend(uint8_t* data, uint8_t dlc, uint8_t deviceNumber) {
	if (dlc > 8) {
		errorCode = 127;
		errorInDeviceNumber = deviceNumber;
		return;
	}
	else {
		if (robotContainer->sniffing())
			messagePrint((*idIn)[deviceNumber], dlc, data, true);
		// if (maximumNumberOfBoards == 8) {
		// 	static int cnt = 0;
		// 	print("Device: %s %i,id: %i, cnt: %i\n\r", name(deviceNumber), deviceNumber,
		// 		(*idIn)[deviceNumber], cnt);
		// 	print("Over\n\r");
		// 	++cnt;
		// 	delay(500);
		// 	if (++cnt > 18)
		// 		return;
		// }
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], dlc, data);
	}
}

/** Returns device's name
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - name
*/
char* Board::name(uint8_t deviceNumber) {
	return (*_name)[deviceNumber];
}

/** Request notification
@param commandRequestingNotification
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::notificationRequest(uint8_t commandRequestingNotification, uint8_t deviceNumber) {
	printf("THIS FUNCTION DOESN'T WORK\n\r");
	//while (1);
	//uint8_t tries = 0;
	//while (tries < 10) {
	//	tries++;
	//	canData[0] = commandRequestingNotification;
	//	messageSend(canData, 1, deviceNumber);
	//	while (tries != 0xFF && !dequeEmpty()) {
	//		robotContainer->noLoopWithoutThis();
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
void Board::oscillatorTest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			oscillatorTest(i);
	else {
		if (alive(deviceNumber)) {
			print("Test %s\n\r", name(deviceNumber));
			canData[0] = COMMAND_OSCILLATOR_TEST;
			messageSend(canData, 1, deviceNumber);
		}
	}
}

/** Reset
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
*/
void Board::reset(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			reset(i);
	else {
		canData[0] = COMMAND_RESET;
		messageSend(canData, 1, deviceNumber);
	}
}


/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param measuringModeNow - Measuring mode id. Default 0.
@param refreshMs - gap between 2 CAN Bus messages to refresh local Arduino copy of device's data. 0 - device's default.
*/
void Board::start(uint8_t deviceNumber, uint8_t measuringModeNow, uint16_t refreshMs) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			start(i, measuringModeNow, refreshMs);
	else {
		if (alive(deviceNumber)) {
			// print("Alive, start reading: %s, mode: %i\n\r", name(deviceNumber), measuringModeNow);
#if REQUEST_NOTIFICATION
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, deviceNumber);
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
			// if (maximumNumberOfBoards == 8) 
			// 	print("In: %i\n\r", deviceNumber);
			messageSend(canData, refreshMs == 0 ? 1 : 3, deviceNumber);
			// if (maximumNumberOfBoards == 8)
			// 	print("Out: %i\n\r", deviceNumber);
#endif
		}
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::stop(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			stop(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			messageSend(canData, 1, deviceNumber);
			(*_lastReadingMs)[deviceNumber] = 0;
		}
	}
}


/** add() assigns device numbers one after another. swap() changes the sequence later. Therefore, add(); add(); will assign number 0 to a device with the smallest CAN Bus id and 1 to the one with the next smallest.
If we want to change the order so that now the device 1 is the one with the smalles CAN Bus id, we will call swap(0, 1); after the the add() commands.
@param deviceNumber1 - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param deviceNumber2 - Second device.
*/
void Board::swap(uint8_t deviceNumber1, uint8_t deviceNumber2) {
	if (deviceNumber1 >= nextFree || deviceNumber2 >= nextFree)
		sprintf(errorMessage, "%s: device overflow: %i or %i", name(), deviceNumber1, deviceNumber2);
	else {
		uint16_t idInTemp = (*idIn)[deviceNumber1];
		uint16_t idOutTemp = (*idOut)[deviceNumber1];
		(*idIn)[deviceNumber1] = (*idIn)[deviceNumber2];
		(*idOut)[deviceNumber1] = (*idOut)[deviceNumber2];
		(*idIn)[deviceNumber2] = idInTemp;
		(*idOut)[deviceNumber2] = idOutTemp;
	}
}


/**
@param robot - robot containing this board
@param devicesOnABoard - number of devices on each board
@param boardName - board's name
@param maxNumberOfBoards - maximum number of boards
@param id - unique id
*/
MotorBoard::MotorBoard(Robot* robot, uint8_t devicesOnABoard, const char* boardName, uint8_t maxNumberOfBoards, BoardId id) :
	Board(robot, maxNumberOfBoards, devicesOnABoard, boardName, MOTOR_BOARD, id) {
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
void MotorBoard::directionChange(uint8_t deviceNumber) {
	if (deviceNumber >= nextFree)
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName, deviceNumber);
	else
		(*reversed)[deviceNumber] = !(*reversed)[deviceNumber];
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool MotorBoard::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint32_t enc = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
					(*encoderCount)[deviceNumber] = enc;
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				}
				default:
					print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 200;
					errorInDeviceNumber = deviceNumber;
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
uint16_t MotorBoard::reading(uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName, deviceNumber);
		return 0;
	}
	alive(deviceNumber, true);
	if (started(deviceNumber))
		return (*encoderCount)[deviceNumber];
	else
		return 0;
}

/** Print all readings in a line
*/
void MotorBoard::readingsPrint() {
	print("Encoders:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %4i", (*encoderCount)[deviceNumber]);
}


/** Motor speed
@param motorNumber - motor's number
@param speed - in range -127 to 127
*/
void MotorBoard::speedSet(uint8_t motorNumber, int8_t speed) {
	if (motorNumber >= nextFree) {
		sprintf(errorMessage, "Mot. %i doesn't exist", motorNumber);
		return;
	}

	if ((*lastSpeed)[motorNumber] == speed)
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
bool MotorBoard::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_MOTORS_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		// print("Start mrm-bldc4x2.5%i \n\r", deviceNumber); 
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					// print("BLDC confirmed\n\r");
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		printf(errorMessage, "%s %i dead.", _boardsName, deviceNumber);
		return false;
	}
	else
		return true;
}

/** Stop all motors
*/
void MotorBoard::stop() {
	for (uint8_t i = 0; i < nextFree; i++) {
		speedSet(i, 0);
		delayMicroseconds(200);
	}
}

/**Test
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param betweenTestsMs - time in ms between 2 tests. 0 - default.
*/
void MotorBoard::test(uint8_t deviceNumber, uint16_t betweenTestsMs)
{
	const uint16_t PAUSE_MS = 20;
	const uint16_t DISPLAY_PAUSE_MS = 300;
	// const uint8_t STEP = 1;
	const uint8_t MAXIMUM_SPEED = 100; // Max. 127

	const int8_t startSpeed[3] = { 0, MAXIMUM_SPEED, -MAXIMUM_SPEED };
	const int8_t endSpeed[3] = { MAXIMUM_SPEED, -MAXIMUM_SPEED, 0 };
	const int8_t step[3] = { 1, -1, 1 };

	// Select motor
	print("%s - enter motor number [0-%i] or wait for all\n\r", name(), nextFree - 1);
	uint16_t selectedMotor = robotContainer->serialReadNumber(3000, 500, nextFree - 1 > 9, nextFree - 1, false);
	if (selectedMotor == 0xFFFF)
		print("Test all\n\r");
	else
		print("\n\rTest motor %i\n\r", selectedMotor);

	// Select speed
	bool fixedSpeed = false;
	print("Enter speed [0-127] or wait for all\n\r");
	uint16_t selectedSpeed = robotContainer->serialReadNumber(2000, 500, false, 127, false);
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
		for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {

			if ((selectedMotor != 0xFFFF && motorNumber != selectedMotor) || !alive(motorNumber))
				continue;

			if (!encodersStarted[motorNumber]) {
				encodersStarted[motorNumber] = true;
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
				messageSend(canData, 1, motorNumber);
			}

			if (fixedSpeed) {
				speedSet(motorNumber, selectedSpeed);
				if (robotContainer->userBreak())
					goOn = false;
				robotContainer->delayMs(PAUSE_MS);
				continue;
			}

			for (uint8_t group = 0; group < 3; group++)
				for (int16_t speed = startSpeed[group];
				((step[group] > 0 && speed <= endSpeed[group]) || (step[group] < 0 && speed >= endSpeed[group])) && goOn;
					speed += step[group]) {
				//blink();
				if (robotContainer->userBreak())
					goOn = false;

				speedSet(motorNumber, speed);

				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("Mot. %i:%3i, en: %i\n\r", motorNumber, speed, (*encoderCount)[motorNumber]);
					lastMs = millis();
				}
				robotContainer->delayMs(PAUSE_MS);
			}

			speedSet(motorNumber, 0);
		}
	}

	// Stop all motors
	for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {
		speedSet(motorNumber, 0);

		if (encodersStarted[motorNumber]) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			messageSend(canData, 1, motorNumber);
		}
	}
}




/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param devicesOnABoard - number of devices on each board
@param boardName - board's name
@param maxNumberOfBoards - maximum number of boards
@param id - unique id
*/
SensorBoard::SensorBoard(Robot* robot, uint8_t devicesOnABoard, const char boardName[], uint8_t maxNumberOfBoards, 
	BoardId id, uint8_t readingsCount) :
	Board(robot, maxNumberOfBoards, devicesOnABoard, boardName, SENSOR_BOARD, id) {
		_readingsCount = readingsCount;
}

/** Starts periodical CANBus messages that will be refreshing values that mirror sensor's calculated values
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::continuousReadingCalculatedDataStart(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingCalculatedDataStart(i);
	else {
		if (alive(deviceNumber)) {
			//print("Alive, start reading: %s\n\r", name(deviceNumber));
#if REQUEST_NOTIFICATION // todo
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, deviceNumber);
#else
			canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA;
			messageSend(canData, 1, deviceNumber);
			//robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 1, canData);
			//print("Sent to 0x%x\n\r, ", (*idIn)[deviceNumber]);
#endif
		}
	}
}

/** Standard deviation
@param sampleCount - count.
@param sample - values.
@param averageValue - output parameter.
@return - standard deviation.*/
float SensorBoard::stardardDeviation(uint8_t sampleCount, uint16_t sample[], float * averageValue){
				// Average and standard deviation
			float sum = 0.0;
			for(uint8_t i = 0; i < sampleCount; i++)
				sum += sample[i];
			//print("Sum %i\n\r", (int)sum);
			*averageValue = sum / sampleCount;
			//print("Mean %i\n\r", (int)mean);
			float sd = 0.0;
			for(int i = 0; i < sampleCount; i++) 
				sd += pow(sample[i] - *averageValue, 2);
			sd = sqrt(sd / sampleCount);
			//print("SD %i\n\r", (int)standardDeviation);
			return sd;
}


/** Filter out data outliers and return average of the rest
@param sampleCount - count.
@param sample - values.
@param averageValue - average value.
@param sigmaCount - number of sigmas to keep.
@param standardDeviation - standard deviation.
@return average value of the filtered set*/
float SensorBoard::outlierlessAverage(uint8_t sampleCount, uint16_t sample[], float averageValue, uint8_t sigmaCount,
	float standardDeviation){
	// Filter out all the values outside n-sigma boundaries and return average value of the rest
	float sum = 0;
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < sampleCount; i++)
		if (averageValue - sigmaCount * standardDeviation < sample[i] && 
				sample[i] < averageValue + sigmaCount * standardDeviation){
			sum += sample[i];
			cnt++;
		}
	return (uint16_t)(sum / cnt);
}

MotorGroup::MotorGroup(Robot* robot){
	this->robotContainer = robot;
}

/** Angle between -180 and 180 degrees
@return - angle
*/
float MotorGroup::angleNormalized(float angle) {
	if (angle < -180)
		angle += 360;
	else if (angle > 180)
		angle -= 360;
	return angle;
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
MotorGroupDifferential::MotorGroupDifferential(Robot* robot, MotorBoard* motorBoardForLeft1, uint8_t motorNumberForLeft1, MotorBoard* motorBoardForRight1, uint8_t motorNumberForRight1,
	MotorBoard* motorBoardForLeft2, uint8_t motorNumberForLeft2, MotorBoard * motorBoardForRight2, uint8_t motorNumberForRight2) : MotorGroup(robot) {
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
			//motorBoard[i]->speedSet(motorNumber[i], speeds[i]);
			if (maxSpeed > speedLimit) {
				motorBoard[i]->speedSet(motorNumber[i], (int8_t)(speeds[i] / maxSpeed * speedLimit));
				//Serial.print("MAX ");
			}
			else {
				motorBoard[i]->speedSet(motorNumber[i], (int8_t)speeds[i]);
				//Serial.print((String)speeds[i] + " ");
			}
			robotContainer->delayMs(1);
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
MotorGroupStar::MotorGroupStar(Robot* robot, MotorBoard* motorBoardFor45Degrees, uint8_t motorNumberFor45Degrees, MotorBoard* motorBoardFor135Degrees, uint8_t motorNumberFor135Degrees,
	MotorBoard* motorBoardForMinus135Degrees, uint8_t motorNumberForMinus135Degrees, MotorBoard* motorBoardForMinus45Degrees, uint8_t motorNumberForMinus45Degrees) : MotorGroup(robot){
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
				robotContainer->delayMs(1);
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
