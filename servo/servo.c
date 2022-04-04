#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_servos.h>

__EXPORT int servo(int argc, char *argv[]);

int servo(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	int sensor_sub_fd = orb_subscribe(ORB_ID(actuator_servos));// Подписываемся на данные датчиков сервопривода.
	
	orb_set_interval(sensor_sub_fd, 200);// Ограничиваем частоту обновления до 5 Гц.

	// Структура для публикации данных датчиков.
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = 
	{
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 1; i++) 
	{
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) 
		{
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) 
		{
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) 
			{
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} 
		else 
		{

			if (fds[0].revents & POLLIN)
		{
			// Полученные данные для первого файлового дескриптора.
			struct actuator_servos_s servo;
			// Копируем необработанные данные датчиков в локальный буфер.
			orb_copy(ORB_ID(actuator_servos), sensor_sub_fd, &servo);
			//Сообщение управления сервоприводом
				//uint64 timestamp - время с момента запуска системы(микросекунды)
				//uint64 timestamp_sample - временная метка данных, на которой основан этот управляющий ответ, была выбрана

				//uint8 NUM_CONTROLS = 8
				//float32[8] control - range: [-1, 1] , где 1 означает максимальную положительную позицию,
														// -1 максимальное отрицательное значение,
														// и карты NaN для обезвреживания
			PX4_INFO("Servo:\t%8.4f", (double)servo.control[0]);

			att.q[0] = 1/*servo.control[0]*/;

			orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);

		}
		}
	}

	PX4_INFO("exiting");

	return 0;
}
