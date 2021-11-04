#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class Gimbal {
	public:
		Gimbal();
		can_frame get_feedback_frame();
        int get_received_angle();
        int get_received_total_angle();
        int get_speed();
        int get_mode();
        double get_angle();
        double get_total_angle();
        double get_rpm();
        double get_volt();
        double get_current();
        double get_temperature();

        void fix_coder(); 
        void set_origin();
        void read_position_speed();
        void read_status();
        void clear_error();

		void turn_off();
        void return_origin();   // don't understand what this does
        void return_shortest();
        void open_loop(int16_t speed);
		void speed_close_loop(int16_t speed);
        void absolute_position_close_loop(u_int32_t position);
        void relative_position_close_loop(int16_t position);
        void motor_read_speed();
        void motor_set_speed(int16_t speed);    // set speed for position_close_loop movement
		int socket_close();
        void print_feedback();

	private:
		struct can_frame send_frame;
		struct can_frame feedback_frame;

		int received_angle, received_total_angle, speed, mode;
        double angle, total_angle, rpm, volt, current, temperature;

		int s, nbytes;
		struct sockaddr_can addr;
		struct ifreq ifr;

		int motor_write();
		int motor_read();
	
};

Gimbal::Gimbal() {
	received_angle = received_total_angle = speed = mode = 0;
    angle = total_angle = rpm = volt = current = temperature = 0;

	send_frame.can_id = 0x00;
	send_frame.can_dlc = 0;
	feedback_frame.can_id = 0x00;
	feedback_frame.can_dlc = 8;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
	}

	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
	}
}

can_frame Gimbal::get_feedback_frame()
{
    return feedback_frame;
}

int Gimbal::get_received_angle() { return received_angle; }
int Gimbal::get_received_total_angle() { return received_total_angle; }
int Gimbal::get_speed() { return speed; }
int Gimbal::get_mode() { return mode; }
double Gimbal::get_angle() { return angle; }
double Gimbal::get_total_angle() { return total_angle; }
double Gimbal::get_rpm() { return rpm; }
double Gimbal::get_volt() { return volt; }
double Gimbal::get_current() { return current; }
double Gimbal::get_temperature() { return temperature; }

void Gimbal::fix_coder()
{
    send_frame.can_id = 0x201;
	send_frame.can_dlc = 0;

	motor_write();
}

void Gimbal::set_origin()
{
    send_frame.can_id = 0x211;
	send_frame.can_dlc = 0;

	motor_write();
}

void Gimbal::read_position_speed()
{
    send_frame.can_id = 0x2F1;
	send_frame.can_dlc = 0;

	motor_write();

    received_angle = feedback_frame.data[1] << 8 | feedback_frame.data[0];
    received_total_angle = feedback_frame.data[5] << 24 | feedback_frame.data[4] << 16 | feedback_frame.data[3] << 8 | feedback_frame.data[2];
    speed = feedback_frame.data[7] << 8 | feedback_frame.data[6];

    angle = received_angle * 360 / 16384.0;
    total_angle = received_total_angle * 360 / 16384.0;
    rpm = speed / 10.0;
}

void Gimbal::read_status()
{
    send_frame.can_id = 0x401;
	send_frame.can_dlc = 0;

	motor_write();

    volt = feedback_frame.data[0] * 0.2;
    current = feedback_frame.data[1] * 0.03;
    temperature = feedback_frame.data[2] * 0.4;
    mode = feedback_frame.data[4];
}

void Gimbal::turn_off()
{
	send_frame.can_id = 0x501;
	send_frame.can_dlc = 0;

	motor_write();
}

void Gimbal::return_origin()
{
    send_frame.can_id = 0x511;
    send_frame.can_dlc = 0;

    motor_write();
}

void Gimbal::return_shortest()
{
    send_frame.can_id = 0x521;
    send_frame.can_dlc = 0;

    motor_write();
}

void Gimbal::open_loop(int16_t speed)
{
    send_frame.can_id = 0x531;
	send_frame.can_dlc = 2;

	int higher = speed >> 8;
	int lower = higher << 8 ^ speed;

	send_frame.data[0] = lower;
	send_frame.data[1] = higher;

	motor_write();
}

void Gimbal::speed_close_loop(int16_t speed)
{
	send_frame.can_id = 0x541;
	send_frame.can_dlc = 2;

	int higher = speed >> 8;
	int lower = higher << 8 ^ speed;

	send_frame.data[0] = lower;
	send_frame.data[1] = higher;

	motor_write();
}

void Gimbal::absolute_position_close_loop(u_int32_t position)
{
	send_frame.can_id = 0x551;
	send_frame.can_dlc = 4;

	int higher = position >> 16;
	int lower = higher << 16 ^ position;

    int higher1 = higher >> 8;
    int higher2 = higher1 << 8 ^ higher;

    int lower1 = lower >> 8;
    int lower2 = lower1 << 8 ^ lower;

	send_frame.data[0] = lower2;
	send_frame.data[1] = lower1;
    send_frame.data[2] = higher2;
    send_frame.data[3] = higher1;

    // std::cout << lower2 << lower1 << higher2 << higher1 << std::endl;

	motor_write();
}

void Gimbal::relative_position_close_loop(int16_t position)
{
	send_frame.can_id = 0x561;
	send_frame.can_dlc = 2;

	int higher = position >> 8;
	int lower = higher << 8 ^ position;

	send_frame.data[0] = lower;
	send_frame.data[1] = higher;

	motor_write();
}

void Gimbal::motor_read_speed()
{
    send_frame.can_id = 0x571;
	send_frame.can_dlc = 3;

    send_frame.data[0] = 0;
    send_frame.data[1] = 0;
    send_frame.data[2] = 0;

    motor_write();
}

void Gimbal::motor_set_speed(int16_t speed)
{
    send_frame.can_id = 0x571;
	send_frame.can_dlc = 3;

	int higher = speed >> 8;
	int lower = higher << 8 ^ speed;
    
    send_frame.data[0] = 1;
	send_frame.data[1] = lower;
	send_frame.data[2] = higher;

	motor_write();
}

int Gimbal::motor_write()
{
	if (write(s, &send_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write");
		return 1;
	}

    motor_read();
    return 0;
}

int Gimbal::motor_read()
{
	nbytes = read(s, &feedback_frame, sizeof(struct can_frame));

	if (nbytes < 0) {
		perror("Read");
		return 1;
	}

    return 0;
}

int Gimbal::socket_close()
{
	if (close(s) < 0) {
		perror("Close");
		return 1;
	}

    return 0;
}

void Gimbal::print_feedback()
{
    printf("0x%03X [%d] ",feedback_frame.can_id, feedback_frame.can_dlc);

	for (int i = 0; i < feedback_frame.can_dlc; i++)
	printf("%02X ",feedback_frame.data[i]);

	printf("\r\n");
}

int main()
{
    Gimbal motor;
    motor.return_shortest();
    sleep(1);

    motor.motor_set_speed(1000);
    motor.absolute_position_close_loop(9000);
    sleep(1);

    motor.absolute_position_close_loop(0);
    sleep(1);


    motor.turn_off();
    motor.socket_close();
    return 0;
}