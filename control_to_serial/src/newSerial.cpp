#include <ros/ros.h> 

#include <serial/serial.h>

#include <std_msgs/String.h>

#include <std_msgs/Empty.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

serial::Serial ser;

int i;

int j;

uint8_t receive_data[1000];

int receive_data_cnt = 0;

// recevie_data_value

uint8_t contorl_mode_r;

uint8_t e_stop_r;

uint8_t gear_r;

uint16_t speed_r;

int16_t steer_r;

uint8_t brake_r;

int enc_r;

/////////////////////////

// transmit_data_value

uint8_t contorl_mode_t = 1;

uint8_t e_stop_t = 0;

uint8_t gear_t = 0;

uint16_t speed_t = 0;

int16_t steer_t = 0;

uint8_t brake_t = 1;

uint8_t alive_t = 0;

// steer value subscribe callback function

void steer_callback(const std_msgs::Float64::ConstPtr& msg)

{

    steer_t = int(msg -> data * 71); 

    //if angle is reverse use under command instead of up command

    // steer_t = -(float_msgs -> data * 71); 

}

void speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
    speed_t = uint16_t(msg -> data)*10;
    printf("1234123412341234\n");
    
}
void brake_callback(const std_msgs::Float64::ConstPtr& msg)
{
    brake_t = (msg -> data);
}
void gear_callback(const std_msgs::Float64::ConstPtr& msg)
{
    gear_t = (msg -> data);
}

int main (int argc, char** argv){
    //Serial코드 내에서 속도, 조향, 브레이크, 기어 수정 xxxxxxxxxxxxxxxxxxxx
    //SubScribe 수정 xxxxxxxxxxxxxxxxxxxxxxx
    ros::init(argc, argv, "ERP42_serial_contorl_for_vision");

    ros::NodeHandle nh;

    // steer value subscribe
    
    ros::Subscriber steering_sub = nh.subscribe<std_msgs::Float64>("steer", 10, steer_callback);
    ros::Subscriber speed_sub = nh.subscribe<std_msgs::Float64>("speed", 10, speed_callback);
    ros::Subscriber brake_sub = nh.subscribe<std_msgs::Float64>("brake", 10, brake_callback);
    ros::Subscriber gear_sub = nh.subscribe<std_msgs::Float64>("gear", 10, gear_callback);

    try

    {

        ser.setPort("/dev/ttyUSB0");

        ser.setBaudrate(115200);

        serial::Timeout to = serial::Timeout::simpleTimeout(1000);

        ser.setTimeout(to);

        ser.open();

    }

    catch (serial::IOException& e)

    {

        ROS_ERROR_STREAM("Unable to open port ");

        return -1;

    }

    if(ser.isOpen()){

        ROS_INFO_STREAM("Serial Port initialized");

    }else{

        return -1;

    }

    ros::Rate loop_rate(50);

    while(ros::ok()){

        ros::spinOnce();

        

        if(ser.available())

        {

            // ERP42_data_receive_start-----------------------------------------------------------------------------------------------------

            std_msgs::String result;

            result.data = ser.read(ser.available());

            for(i=0;i<sizeof(result.data);i++)

            {

                receive_data[i+receive_data_cnt] = uint8_t(result.data[i]);

            }

            receive_data_cnt += i;

            if(receive_data_cnt >= 18)

            {

                for(i=0;i < receive_data_cnt - 3;i++)

                {

                    if(receive_data[i] == 0x53 & receive_data[i+1] == 0x54 & receive_data[i+2] == 0x58)

                    {

                        if(receive_data_cnt > i + 17)

                        {

                            if(receive_data[i+16] == 0x0D & receive_data[i+17] == 0x0A)

                            {

                                // system("clear");

                                // ROS_INFO_STREAM("DATA_RECEIVED~!!!!!!!!!!!!!!!!!");

                                receive_data_cnt = 0;

                                /*

                                receive_data[i] -> S

                                receive_data[i+1] -> T

                                receive_data[i+2] -> X

                                receive_data[i+3] -> A or M (0 : A / 1 : M)

                                receive_data[i+4] -> E-stop (0 : OFF / 1 : ON)

                                receive_data[i+5] -> Gear (0 : forward / 1 : natural / 2 : backward)

                                receive_data[i+6] -> speed_0 (0 ~ 200 : real_speed[km/h] * 10)

                                receive_data[i+7] -> speed_1

                                receive_data[i+8] -> steer_0 (-2000 ~ 2000 : real_steer * 71)

                                receive_data[i+9] -> steer_1

                                receive_data[i+10] -> brake (1 ~ 200)

                                receive_data[i+11] -> enc_0 (-2^31 ~ 2^31 )

                                receive_data[i+12] -> enc_1

                                receive_data[i+13] -> enc_2

                                receive_data[i+14] -> enc_3

                                receive_data[i+15] -> alive

                                receive_data[i+16] -> 0x0D

                                receive_data[i+17] -> 0x0A

                                */

                                contorl_mode_r = receive_data[i+3];

                                e_stop_r = receive_data[i+4];

                                gear_r = receive_data[i+5];

                                speed_r = receive_data[i+7] * 256 + receive_data[i+6];

                                steer_r = receive_data[i+9] * 256 + receive_data[i+8];

                                brake_r = receive_data[i+10];

                                enc_r = (receive_data[i+14] << 24) + (receive_data[i+13] << 16) + (receive_data[i+12] << 8) + receive_data[i+11];

                                // printf("control_mode : %d | e_stop : %d | gear_r : %d | speed_r : %d | steer_r : %d |brake_r : %d | enc_r : %d", contorl_mode_r, e_stop_r, gear_r, speed_r, steer_r, brake_r, enc_r);

                                for(i=0;i<receive_data_cnt;i++)

                                {

                                    receive_data[i] = 0;

                                }

                                break;

                            }

                        }

                    }

                }

            }

        }

        // ERP42_data_receive_end-----------------------------------------------------------------------------------------------------

        contorl_mode_t = 1;

        e_stop_t = 0;

        gear_t = 0;

        // speed_value ex)3km/h = 30,  5km/h = 50

        // speed_t = 20;

        // steer_t = 0;

        // brake_t = 1;

        

        uint8_t transmit[14];

        transmit[0] = 'S';

        transmit[1] = 'T';

        transmit[2] = 'X';

        transmit[3] = contorl_mode_t;

        transmit[4] = e_stop_t;

        transmit[5] = gear_t;

        transmit[6] = speed_t / 256;

        transmit[7] = speed_t % 256;

        transmit[8] = steer_t / 256;

        transmit[9] = steer_t % 256;

        transmit[10] = brake_t;

        transmit[11] = alive_t;

        transmit[12] = 0x0D;

        transmit[13] = 0x0A;

        ser.write(transmit, 14);

        alive_t++;

        alive_t%=256;

        

        for(i=0;i<14;i++)

        {

         //   printf("| %x |", transmit[i]);

        }

        //printf("\n");
        printf("speed : %d\n", speed_t);
        printf("steer : %d\n", steer_t);
    }

    loop_rate.sleep();

}