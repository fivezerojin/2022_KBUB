#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

double Arr[10000][2];
int num_of_coord = 0;
void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_array_test");

	ros::NodeHandle n;	

	ros::Subscriber sub3 = n.subscribe("odom_array", 10, arrayCallback);

	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		for(int j = 0; j < int(num_of_coord/2); j++)
		{
			for(int k = 0; k < 2; k++)
				if(k == 0)
					printf("%lf, ", Arr[j][k]);
				else
					printf("%lf\n", Arr[j][k]);
		}
		printf("------------\n");
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}

void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
	num_of_coord = 0;
	int x = 0;
	int y = 0;
	while(true)
	{
		Arr[x][y] = array->data[num_of_coord++];
		if(y == 0)	y++;
		else{
			x++;
			y--;
		}
		
		if(array->data[num_of_coord] < 0.1)	break;
	}

	return;
}

