/*
 ============================================================================
 Name        : Project.c

 Author      : Marta Rguez Ramos and Alaa Salih
 Version     :
 Copyright   : Your copyright notice
 Description : Gyroscope and color sensor
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int file_gyro;
int file_rgb;


#define time 0.01//period 10ms (100Hz)
#define g_sens 0.07 //Sensitivity gyroscope (2000dps)

int main() {
	char *bus_rgb = "/dev/i2c-1";
	char *bus_gyro = "/dev/i2c-2";

	if ((file_rgb = open(bus_rgb, O_RDWR)) < 0) {
		printf("Failed to open the rgb bus. \n");
		exit(1);
	}

	if ((file_gyro = open(bus_gyro, O_RDWR)) < 0) {
		printf("Failed to open the gyro bus. \n");
		exit(1);
	}

	printf("Rgb bus opened. Return value = %d \n",
			ioctl(file_rgb, I2C_SLAVE, 0x29));

	printf("Gyro bus opened. Return value = %d \n",
			ioctl(file_gyro, I2C_SLAVE, 0x6B));

	config_rgb();
	config_gyro();

	float xangle_accum = 0.0f;
	float yangle_accum = 0.0f;
	float zangle_accum = 0.0f;

	while (1) {

		char reg[1] = {0x27};
		char data[1]={0};
		write(file_gyro, reg, 1);
		read(file_gyro, data, 1);
		//Print rotational velocity and angular displacement
		if (data[0] & 0x7) {
			 get_gyro_data(data[0], &xangle_accum, &yangle_accum, &zangle_accum);
		}
		//Print color values
		get_rgb_values();
	}
	exit(0);
}

void get_gyro_data(char reg_status, float *xangle_accum,float *yangle_accum,float *zangle_accum ) {

	// Instantaneous local velocities (x,y and z)
	float xGyro = 0.0f;
	float yGyro = 0.0f;
	float zGyro = 0.0f;
	//check the X-axis
	char reg[1]={0};
	char datai[1] = { 0 };
	char data_found = 0x0;
	if (reg_status & 0x1) {
		data_found = data_found | 0x1;
		reg[0] = 0x28;
		write(file_gyro, reg, 1);
		read(file_gyro, datai, 1);
		char data_0 = datai[0];

		reg[0] = 0x29;
		write(file_gyro, reg, 1);
		read(file_gyro, datai, 1);
		char data_1 = datai[0];

		int new_valuex = (data_1 << 8) + data_0;
		if (new_valuex > 32767) {
			new_valuex -= 65536;
		}

		xGyro = (float) new_valuex * g_sens;
		if (xGyro>-4 && xGyro < 4) {
			xGyro = 0;
		}

		*xangle_accum += (xGyro * time);


		//coordinate system
		if (*xangle_accum < 0 ){
			*xangle_accum += 360;
		}

		if (*xangle_accum > 360 ) {
			*xangle_accum -= 360;
		}

	}
	// check the Y-axis
	if (reg_status & 0x2) {
		data_found = data_found | 0x2;
		//yGyro lsb
		reg[0] = 0x2A;
		write(file_gyro, reg, 1);
		read(file_gyro, datai, 1);
		char data_2 = datai[0];

		//yGyro msb
		reg[0] = 0x2B;
		write(file_gyro, reg, 1);
		read(file_gyro, datai, 1);
		// same operation
		char data_3 = datai[0];

		int new_valuey = (data_3 << 8) + data_2;
		if (new_valuey > 32767) {
			new_valuey -= 65536;
		}

		yGyro = (float) new_valuey * g_sens;

		if (yGyro>-4 && yGyro < 4) {
			yGyro = 0;
		}
		*yangle_accum += yGyro * time;
		if (*yangle_accum < 0 ){
			*yangle_accum += 360.0;
		}

		if (*yangle_accum > 360 ) {
			*yangle_accum -= 360;
		}
	}

	// check the Z-axis
	if (reg_status & 0x4) {
		data_found = data_found | 0x4;
		//zGyro lsb
		reg[0] = 0x2C;
		write(file_gyro, reg, 1);
		read(file_gyro, datai, 1);
		char data_4 = datai[0];

		//zGyro msb
		reg[0] = 0x2D;
		write(file_gyro, reg, 1);
		read(file_gyro, datai, 1);
		char data_5 = datai[0];

		int new_valuez = (data_5 << 8) + data_4;
		if (new_valuez > 32767) {
			new_valuez -= 65536;
		}

		zGyro = (float) new_valuez * g_sens;
		//Drift
		if (zGyro > -4 && zGyro < 4  ) {
			zGyro = 0;
		}

		*zangle_accum += zGyro * time;

		if (*zangle_accum < 0 ){
			*zangle_accum += 360.0;
		}

		if (*zangle_accum > 360 ) {
			*zangle_accum -= 360;
		}
	}
	//print data only if data found
	if (data_found){
		printf("Rotational speed:  x: %.2f\ty: %.2f\tz: %.2f\n", xGyro,yGyro,zGyro);
		printf("Angles:  x: %.2f\ty: %.2f\tz: %.2f\n",*xangle_accum, *yangle_accum, *zangle_accum);
	}
}

void config_rgb() {
	// Select enable register(0x80)
	// Power ON, RGBC enable, wait time disable(0x03)
	char config[2] = { 0 };
	config[0] = 0x80;
	config[1] = 0x03;
	write(file_rgb, config, 2);

	// Select ALS time register(0x81)
	// Atime = 700 ms(0x00)
	config[0] = 0x81;
	config[1] = 0x00;
	write(file_rgb, config, 2);
	// Select Wait Time register(0x83)
	// WTIME : 2.4ms(0xFF)
	config[0] = 0x83;
	config[1] = 0xFF;
	write(file_rgb, config, 2);
	// Select control register(0x8F)k
	// AGAIN = 1x(0x00)
	config[0] = 0x8F;
	config[1] = 0x00;
	write(file_rgb, config, 2);
	sleep(1);
}

void config_gyro() {
	// Enable X, Y, Z-Axis and disable Power down mode(0x0F)
	char config[2] = { 0 };
	config[0] = 0x20;
	config[1] = 0x0F;
	write(file_gyro, config, 2);
	// Full scale range, 2000 dps(0x30)
	config[0] = 0x23;
	config[1] = 0x30;
	write(file_gyro, config, 2);
	sleep(1);
}

void get_rgb_values() {
	// Read 8 bytes of data from register(0x94)
	// cData lsb, cData msb, red lsb, red msb, green lsb, green msb, blue lsb, blue msb
	char reg[1] = {0x94};
	write(file_rgb, reg, 1);
	char data[8] = { 0 };
	if (read(file_rgb, data, 8) != 8) {
		printf("Error : Input/output Error \n");
	} else {
		// Convert the data
		int cData = (data[1] * 256 + data[0]);
		int red = (data[3] * 256 + data[2]);
		int green = (data[5] * 256 + data[4]);
		int blue = (data[7] * 256 + data[6]);

		// Calculate luminance
		float luminance = (-0.32466) * (red) + (1.57837) * (green)
				+ (-0.73191) * (blue);
		if (luminance < 0) {
			luminance = 0;
		}
		printf("--------------------------------------------------------\n");
		// what color is
		printf("RED: %d\n", red);
		printf("BLUE: %d\n", blue);
		printf("GREEN: %d\n", green);
		if (red>8000 && green<10000 && blue>4000){
			printf("Detecting color red \n");
		}
		if (blue>1500 && blue>green && blue>red){
			printf("Detecting color blue \n");
		}
		if (green>1500 && green>blue && green>red){
			printf("Detecting color green \n");
		}
		if (red>8000 && green>6000 && blue<4000) {
			printf("Detecting color orange \n");
		}

		printf("IR : %d [lux] \n", cData);
		printf("Ambient Light : %.2f [lux] \n", luminance);

	}
}


