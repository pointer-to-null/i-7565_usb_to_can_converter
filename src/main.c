#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

#define EXTENDED_CAN_ID_ASCII_SIZE 8
#define CAN_DATA_FULL_SIZE 8
#define CAN_DATA_ASCII_FULL_SIZE 16

#define CAN_DATA_ASCII_Pos 10

#define BYTE_HEX_DIGIT_COUNT 2

#define CAN_FULL_ASCII_FRAME_SIZE 27

#define CONVERTER_EXTENDED_CAN_ID_CODE 0x65

#define WINDOWS_END_OF_LINE_CODE 0x0A
#define LINUX_END_OF_LINE_CODE 0x0D

//Only for extended CAN ID. (CAN ID == 29 bits)
uint32_t parseCANid(const char *received_ascii_msg) {
	if(received_ascii_msg == NULL) {
		return -1;
	}

	char buffer[EXTENDED_CAN_ID_ASCII_SIZE];
	for(int i = 0; i < EXTENDED_CAN_ID_ASCII_SIZE; i++) {
		buffer[i] = received_ascii_msg[i + 1];
	}
	long int can_id_value = strtol(buffer, NULL, 16);
	return (uint32_t)can_id_value;
}

//Only for extended CAN ID with data size == 8 bytes. (extended CAN ID == 29 bits and CAN data == 8 bytes)
void parseCANdata(const char *received_ascii_msg, char *output_data) {
	if(received_ascii_msg == NULL || output_data == NULL) {
		return;
	}

	char buffer[2];
	for(int i = 0, j = 0; i < CAN_DATA_ASCII_FULL_SIZE; i = i + 2, j++) {
		buffer[0] = received_ascii_msg[CAN_DATA_ASCII_Pos + i];
		buffer[1] = received_ascii_msg[CAN_DATA_ASCII_Pos + i + 1];
		output_data[j] = (char)strtol(buffer, NULL, 16);
	}
}

int parseCANdataSize(const char *received_ascii_msg) {
	if(received_ascii_msg == NULL) {
		return -1;
	} else {
		return received_ascii_msg[9] - 0x30;
	}
}

//Only for extended CAN ID (extended CAN ID == 29 bits)
void canIDtoStr(const uint32_t can_id, char *can_id_str) {
	if(can_id_str == NULL) {
		return;
	}

	snprintf(can_id_str, EXTENDED_CAN_ID_ASCII_SIZE + 1, "%08X", can_id);
}

//Only for CAN data size == 8 bytes
void canDataToStr(const char *can_data, char *can_data_str) {
	if(can_data == NULL || can_data_str == NULL) {
		return;
	}

	char buffer[BYTE_HEX_DIGIT_COUNT + 1];
	for(int i = 0, j = 0; i < CAN_DATA_FULL_SIZE; i++, j = j + 2) {
		snprintf(buffer, BYTE_HEX_DIGIT_COUNT + 1, "%02hhX", can_data[i]);
		can_data_str[j] = buffer[0];
		can_data_str[j + 1] = buffer[1];
	}
	can_data_str[CAN_DATA_ASCII_FULL_SIZE] = '\0';
}

//Only for extended CAN ID and CAN data size == 8 byte
void createASCIIcanFrame(char *ascii_can_frame, uint32_t can_id, const char *can_data) {
	if(ascii_can_frame == NULL || can_data == NULL) {
		return;
	}

	char can_id_ascii_str[EXTENDED_CAN_ID_ASCII_SIZE + 1];
	canIDtoStr(can_id, can_id_ascii_str);

	char can_data_ascii_str[CAN_DATA_ASCII_FULL_SIZE + 1];
	canDataToStr(can_data, can_data_ascii_str);

	ascii_can_frame[0] = CONVERTER_EXTENDED_CAN_ID_CODE;
	ascii_can_frame[1] = can_id_ascii_str[0];
	ascii_can_frame[2] = can_id_ascii_str[1];
	ascii_can_frame[3] = can_id_ascii_str[2];
	ascii_can_frame[4] = can_id_ascii_str[3];
	ascii_can_frame[5] = can_id_ascii_str[4];
	ascii_can_frame[6] = can_id_ascii_str[5];
	ascii_can_frame[7] = can_id_ascii_str[6];
	ascii_can_frame[8] = can_id_ascii_str[7];
	ascii_can_frame[9] = 0x38;	//data size

	for(int i = 10, j = 0; i < ((ascii_can_frame[9] - 0x30) * 2) + 10 && j < CAN_DATA_ASCII_FULL_SIZE; i++, j++) {
		ascii_can_frame[i] = can_data_ascii_str[j];
	}
	ascii_can_frame[((ascii_can_frame[9] - 0x30) * 2) + 10] = LINUX_END_OF_LINE_CODE;

}

int openUSBport(const char *path) {
	int port_descr = open(path, O_RDWR);
	if(port_descr < 0) {
		perror("Open USB port.");
		return -1;
	}
	struct termios tty_port;

	int tc_port = tcgetattr(port_descr, &tty_port);
	if(tc_port < 0) {
		perror("tcgetattr");
		return -1;
	}

	cfsetospeed(&tty_port, (speed_t)B921600);
	cfsetispeed(&tty_port, (speed_t)B921600);

	tty_port.c_cflag &= ~PARENB;
	tty_port.c_cflag &= ~CSTOPB;
	tty_port.c_cflag &= ~CSIZE;
	tty_port.c_cflag |= CS8;

	tty_port.c_cflag &= ~CRTSCTS;

	tty_port.c_cflag |= CREAD | CLOCAL;
	tty_port.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty_port.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty_port.c_oflag &= ~OPOST;

	tcflush(port_descr, TCIFLUSH);
	tcsetattr(port_descr, TCSANOW, &tty_port);
	printf("port %s has been opened\n", path);
	return port_descr;
}

void *receive(void *port_descr) {
	if(*(int *)port_descr < 0) {
		return (void *)0;
	}

	char read_char = 0x00;
	char read_buff[CAN_FULL_ASCII_FRAME_SIZE];

	while(1) {
		int read_count = read(*(int *)port_descr, &read_char, 1);
		if(read_char == CONVERTER_EXTENDED_CAN_ID_CODE) {
			read_buff[0] = read_char;
			int i = 1;

			do {
				read_count = read(*(int *)port_descr, &read_char, 1);
				read_buff[i] = read_char;
				i++;
			} while((read_char != WINDOWS_END_OF_LINE_CODE) && (i < CAN_FULL_ASCII_FRAME_SIZE));


			printf("received: ");
			for(int j = 0; j < CAN_FULL_ASCII_FRAME_SIZE; j++) {
				printf("%02X ", read_buff[j]);
			}
			printf("\n");
			printf("can_id = %08X\n", parseCANid(read_buff));

			char can_data[CAN_DATA_FULL_SIZE];
			parseCANdata(read_buff, can_data);
			printf("can_data: ");
			for(int j = 0; j < CAN_DATA_FULL_SIZE; j++) {
				printf("%02hhX ", can_data[j]);
			}
			printf("\n");
			int can_data_size = parseCANdataSize(read_buff);
			printf("can data size = %d\n", can_data_size);
		}
	}
}

void* transmit(void *port_descr) {
	if(*(int *)port_descr < 0) {
		return (void *)0;
	}

	uint32_t can_id = 0x0CF01464;
	char can_id_str[EXTENDED_CAN_ID_ASCII_SIZE + 1];

	canIDtoStr(can_id, can_id_str);
	printf("CAN ID int to str: %s\n", can_id_str);

	char can_data[CAN_DATA_FULL_SIZE] = {0xFF, 0xFF, 0xFF, 0x45, 0x67, 0xFF, 0x55, 0xFF};
	char can_data_str[CAN_DATA_ASCII_FULL_SIZE + 1];

	canDataToStr(can_data, can_data_str);
	printf("CAN DATA int to str: %s\n", can_data_str);

	char can_ascii_full_frame[CAN_FULL_ASCII_FRAME_SIZE];

	createASCIIcanFrame(can_ascii_full_frame, can_id, can_data);

	for(int i = 0; i < 5000; i++) {
		int write_count = write(*(int *)port_descr, can_ascii_full_frame, CAN_FULL_ASCII_FRAME_SIZE);
		printf("bytes write = %d: ", write_count);
		for(int j = 0; j < CAN_FULL_ASCII_FRAME_SIZE; j++) {
			printf("%02X ", can_ascii_full_frame[j]);
		}
		printf("\n");
		usleep(1000 * 123);
	}

	return (void *)0;
}

int main(int argc, char **argv) {
	pthread_t receiver;
	pthread_t transmitter;

	int first_port = 0;
	int second_port = 0;
	first_port = openUSBport("/dev/ttyUSB0");
	//second_port = openUSBport("/dev/ttyUSB1");
	printf("first port = %d second port = %d\n", (unsigned int)first_port, (unsigned int)second_port);

	pthread_create(&receiver, NULL, &receive, &first_port);
	pthread_create(&transmitter, NULL, &transmit, &first_port);

	pthread_join(receiver, NULL);
	//pthread_join(transmitter, NULL);

	close(first_port);
	//close(second_port);
	return 0;
}
