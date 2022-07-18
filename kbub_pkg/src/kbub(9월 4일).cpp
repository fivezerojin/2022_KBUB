#include <iostream>
#include <fstream>
#include "CarController.cpp"
#include "GlobalPathManager.cpp"
#include "InputManager.cpp"
#include "CarController.cpp"
#include "coord.cpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include "kbub_pkg/PointArray.h"
#include "kbub_pkg/ParkPoint.h"
#include "velodyne_filter/PointArray.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <stdlib.h>
#include <cmath>
#include <time.h>

#define DEBUG 0
#define MODE 4 // 1 : 본선, 2 : 예선, 3 : test, 4:장애물 테스트
using namespace std;

InputManager *inputData = InputManager::GetInstance();  // 인지 데이터 저장 클래스
CarController *control = CarController::Get();          // 차량 컨트롤 클래스(스티어링, 속도, 브레이크, 기어)
listCoord listOfCoord;                                  // 좌표 데이터 저장 클래스
int test = 0;                                           // test
int cnt = 1;                                            // txt파일 카운트
int cntTemp = -1;                                       // txt파일 임시저장
int cntTemp_d = -1;
int Mission = 0;                                        // 현재 미션 0:x, 1:신호등_직진, 2:신호등_좌회전 3:정적소형, 4:정적대형, 5:동적, 6:주차, 7:배달
bool flag = true;                                       // Stanley 통신용
bool isPark = false;                                    // 주차 완료 확인
bool checkBigObject = false;                            // 대형 장애물 확인
double parkingSign = 0;
int lidarFlag = 0;
bool isGPS = false;                                     // 현재 gps받았는지 여부
bool canPark = false;
bool lidarStopA = false;
double deli_time = 0.0;
bool timeStart = false;
int traffic_count[3] = {0,0,0};
int bigLidarCount = 0;
//ros::Time testTime = ros::Time::now();
void odomPub(ros::Publisher odom_pub);
void flagPub(ros::Publisher stanleyFlag, bool _flag);
void serialPub(ros::Publisher speedPub, ros::Publisher steerPub, ros::Publisher brakePub, ros::Publisher gearPub);

void LaneDetection_cb(const std_msgs::Float64::ConstPtr &msg);
void VisionDistance_cb(const std_msgs::Int16::ConstPtr &msg);
void VisionYoloClass_cb(const std_msgs::Int32MultiArray::ConstPtr &msg);
void ParkingAngle_cb(const std_msgs::Float64::ConstPtr &msg);
void DynamicObject_cb(const std_msgs::Bool::ConstPtr &msg);
void BigObject_cb(const std_msgs::Bool::ConstPtr &msg);
void Odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
void finishEnc_cb(const std_msgs::Bool::ConstPtr &msg);
void smallObject_cb(const std_msgs::Int16::ConstPtr &msg);
void lidarSteer_cb(const std_msgs::Float32::ConstPtr &msg);
void canPark_cb(const std_msgs::Bool::ConstPtr &msg);
void lidarA_cb(const std_msgs::Bool::ConstPtr &msg);
void setMission(ros::Publisher missionPub);
void MissionControl();
void printMission();
void EStop();
void SetDrive();

double distance(double x1, double y1, double x2, double y2);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kbub");
    ros::NodeHandle n;

    ros::Subscriber finishEnc = n.subscribe("/encoder_finish", 10, finishEnc_cb);//엔코더 종료 여부(bool)

    ///Vision
    ros::Subscriber visionLaneDetection = n.subscribe("/LaneDetection", 10, LaneDetection_cb);//미사용??
    ros::Subscriber visionDistance = n.subscribe("/StopLineDistance", 10, VisionDistance_cb);//정지선 거리(int16)
    ros::Subscriber visionYoloClass = n.subscribe("/traffic_sign", 10, VisionYoloClass_cb);//욜로욜로욜로(int)
    ///Lidar
    ros::Subscriber dynamicObject = n.subscribe("/Dynamic_Stop", 10, DynamicObject_cb);//동적 장애물 & 배달(bool) 
    ros::Subscriber bigObject = n.subscribe("/big_object", 10, BigObject_cb);//정적 대형 장애물(bool)
    ros::Subscriber smallObject = n.subscribe("/lidarFlag", 10, smallObject_cb);//정적 소형 장애물(bool)
    ros::Subscriber lidarSteer = n.subscribe("/lidarSteer", 10, lidarSteer_cb);
    ros::Subscriber canPark_sub = n.subscribe("/Lidar_Stop", 10, canPark_cb);
    ros::Subscriber lidarA_sub = n.subscribe("/Lidar_Stop1", 10, lidarA_cb);
    ///Local
    ros::Subscriber odometry = n.subscribe("/odom", 10, Odometry_cb); //현재 gps
    ////////
    /* Publisher */
    ///Stanley
    ros::Publisher stanleyFlag = n.advertise<std_msgs::Bool>("Flag", 10); //Flag  0:kbub->Serial & 1:Stanley->Serial
    ros::Publisher odom_pub = n.advertise<std_msgs::Float64MultiArray>("odom_array", 50); //현재 위치에서 가장 가까운 점과 그앞 20개 점(Float64Array)
    ///Serial
    ros::Publisher speedPub = n.advertise<std_msgs::Float64>("speedP", 10);
    ros::Publisher steerPub = n.advertise<std_msgs::Float64>("steerP", 10);
    ros::Publisher brakePub = n.advertise<std_msgs::Float64>("brakeP", 10);
    ros::Publisher gearPub = n.advertise<std_msgs::Float64>("gearP", 10);

    ros::Publisher missionPub = n.advertise<std_msgs::Int16>("mission", 10);//현재 미션 0:x, 1:신호등_직진, 2:신호등_좌회전 3:정적소형, 4:정적대형, 5:동적, 6:주차, 7:배달
    ros::Rate loop_rate(50);

    //gps좌표 모두 읽어옴
    listOfCoord.readText();
    SetDrive(); // 기본 속도 설정(6.0)

    //test
    if (DEBUG){
        inputData->setGcs(346760.345652,4070386.76388); //gps 임의 설정
        isGPS = true;
    }
    while (ros::ok())
    {
        //test
        if(isGPS)
        {
            setMission(missionPub);//미션설정
            MissionControl();
            odomPub(odom_pub);          // 목표 gps Publish
            flagPub(stanleyFlag, flag); // Flag Publish(Stanley)
        }
        /////test
        if (DEBUG)
        {
            control->SetWheelSpeed((double)test / 3.0);
            if (++test > 1000)    test = 0;
            if(test > 200)  {
                inputData->setBigObject(true);
            }
            if(test > 500)
            {
                inputData->setBigObject(false);
                test = 0;
            }
        
        }
        ////
        if (!flag) // Stanley에서 publish 차단, kbub -> Serial
            serialPub(speedPub, steerPub, brakePub, gearPub);
        printMission(); //현재 미션 프린트
        
        ros::spinOnce();
        loop_rate.sleep();
        system("clear");
    }
    ros::spin();

    printf("finish\n");
    return 0;
}

void odomPub(ros::Publisher odom_pub)
{
    std_msgs::Float64MultiArray array;
    array.data.clear();
    double min = 210000000;
    int min_index = -1;
    //현재 gps좌표 확인
    double x = inputData->getGcs_x();
    double y = inputData->getGcs_y();

    //가장 가까운 좌표 탐색
    for (int i = 0; i < listOfCoord.n[cnt]; i++)
    {
        double dis = distance(x, y, listOfCoord.coordList[cnt][i][0], listOfCoord.coordList[cnt][i][1]);
        if (min > dis)
        {
            min = dis; // 가장 가까운 좌표의 거리
            min_index = i; // 가장 가까운 좌표의 인덱스
        }
    }
    //가장 가까운 좌표부터 총 20개의 좌표 저장, 전송
    printf("---Publish Odom---\n");
    bool isFull = false;
    int j = 0;
    for (int i = min_index; i < min_index + 20; i++) //20개 저장
    {
        if(isFull) // cnt txt파일의 끝에 도달한 경우 다음 텍스트 파일을 읽어옴
        {
            array.data.push_back(listOfCoord.coordList[cnt+1][j][0]);
            array.data.push_back(listOfCoord.coordList[cnt+1][j][1]);
            printf("%lf, %lf\n", listOfCoord.coordList[cnt+1][j][0], listOfCoord.coordList[cnt][j++][1]);
        }
        else //cnt txt파일의 끝에 도달하지 않은경우
        {
            array.data.push_back(listOfCoord.coordList[cnt][i][0]);
            array.data.push_back(listOfCoord.coordList[cnt][i][1]);
            printf("%lf, %lf\n", listOfCoord.coordList[cnt][i][0], listOfCoord.coordList[cnt][i][1]);
        }
        
        if(listOfCoord.coordList[cnt][i+1][0] < 10.2)    isFull = true; //txt파일의 끝에 도달했는지 검사
    }
    printf("cur_index : %d / %d, %d.txt, distance : %lf\n", min_index, listOfCoord.n[cnt], cnt, min);
    printf("------------------\n");
    //////test
    if (DEBUG)
    {
        if (test % 100 == 0)
        {
            if(listOfCoord.coordList[cnt][min_index + 2][0] > 1.1)  
                inputData->setGcs(listOfCoord.coordList[cnt][min_index + 2][0], listOfCoord.coordList[cnt][min_index + 2][1]);
            else
                inputData->setGcs(listOfCoord.coordList[cnt+1][0][0], listOfCoord.coordList[cnt+1][0][1]);
        }
    }
    //////

    //cnt번 txt파일의 데이터를 모두 읽었으면 cnt증가 -> 다음 txt파일로 이동
    if ((listOfCoord.coordList[cnt][min_index + 1][0] == 1.1 && cnt < listOfCoord.end) && cnt != 0 ||isFull)
    {
        printf("c : %lf,\n",listOfCoord.coordList[cnt][min_index + 1][0]);
        cnt++;
    }

    //gps 퍼블리시
    odom_pub.publish(array);

    ros::spinOnce();
}
void flagPub(ros::Publisher stanleyFlag, bool _flag)
{
    //현재 플래그 퍼블리시
    std_msgs::Bool f;
    f.data = _flag;
    stanleyFlag.publish(f);
    ros::spinOnce();
}
void serialPub(ros::Publisher speedPub, ros::Publisher steerPub, ros::Publisher brakePub, ros::Publisher gearPub)
{
    //플래닝에서 직접 보내줘야하는경우 (flag == false)
    std_msgs::Float64 Speed, Steer, Brake, Gear;
    if(Mission != 6){ // 주차 미션의 경우 비전, 제어, 라이다가 직접 해결
    
        Speed.data = control->GetWheelSpeed();
        Steer.data = control->GetWheelAngle();
        Brake.data = control->GetBrake();
        Gear.data = control->GetGear();
        speedPub.publish(Speed);
        steerPub.publish(Steer);
        brakePub.publish(Brake);
        gearPub.publish(Gear);
    }
    ros::spinOnce();
}

void LaneDetection_cb(const std_msgs::Float64::ConstPtr &msg)
{
    inputData->setLaneDistance(msg->data);
}
void VisionDistance_cb(const std_msgs::Int16::ConstPtr &msg)
{
    inputData->setStopDistance(msg->data);
}
void VisionYoloClass_cb(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    for(int i = 0; i < 3; i++)
    {
        if(msg->data[i])    traffic_count[i]++;
        else    traffic_count[i] --;

        if(traffic_count[i]<0)  traffic_count[i] = 0;
        else if(traffic_count[i] > 10)   traffic_count[i] = 10;

        if(traffic_count[i] > 4)
        {
            inputData->setVisionClass(i, true);
        }
        else   inputData->setVisionClass(i, false);
        printf("traffic count %d : %d\n", i, traffic_count[i]);
    }

}
void DynamicObject_cb(const std_msgs::Bool::ConstPtr &msg)
{
    inputData->setDynamicObject(msg->data);
}
void BigObject_cb(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        bigLidarCount++;
        if(bigLidarCount > 10)
            bigLidarCount = 10;
    }
    else
    {
        bigLidarCount--;
        if(bigLidarCount < 0)
            bigLidarCount = 0;
    }
    if(bigLidarCount > 5)
        inputData->setBigObject(true);
    else
        inputData->setBigObject(false);
}
void Odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    double x, y;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    inputData->setGcs(x, y);
    isGPS = true;
}
void smallObject_cb(const std_msgs::Int16::ConstPtr &msg)
{
    lidarFlag = msg->data;
}
void lidarSteer_cb(const std_msgs::Float32::ConstPtr &msg)
{
    if(msg->data < -28.0)
        control->SetWheelAngle(-28.0);
    else if(msg->data  > 28.0)
        control->SetWheelAngle(28.0);
    else
        control->SetWheelAngle(msg->data);
}

void finishEnc_cb(const std_msgs::Bool::ConstPtr &msg)
{
    isPark=msg->data;
}
void canPark_cb(const std_msgs::Bool::ConstPtr &msg)
{
    canPark = msg->data;
}
void lidarA_cb(const std_msgs::Bool::ConstPtr &msg)
{
    lidarStopA = msg->data;
}
double distance(double x1, double y1, double x2, double y2)
{
    //거리 함수
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

void setMission(ros::Publisher missionPub)
{
    std_msgs::Int16 mission_;
    /*
    * MISSION => 0:x, 1:신호등직진, 2:신호등 좌회전, 3:정적소형, 4:정적대형, 5:동적, 6:주차, 7:배달A, 8:배달B
    * cnt => txt파일 숫자!
    * if(cnt ==0) --> 정적 대형 미션 옆차로 gps!
    */
    
    if(MODE == 1)
    {
        if(cnt == 2)
        {
            //주차 진행중일경우 미션 6번 주차 종료될경우 미션 0번!
            if(!isPark)
                Mission = 6; //PAKRING
                
            else
                Mission = 0;
        }  
        else if(cnt == 3 || cnt == 4 || cnt == 6 || cnt == 10 || cnt == 18 || cnt == 20 || cnt == 22 || cnt == 13)   Mission = 1; // 신호등 직진
        else if(cnt == 5 || cnt == 99)   Mission = 4; //정적대형
        else if(cnt == 7 || cnt == 9)   Mission = 7;//배달(7 : 받기, 9 : 배달하기, 구분해야할지 확인)
        else if(cnt == 14)  Mission = 2; //신호등 좌회전
        else Mission = 0;
    }
    else if (MODE == 2)
    {
        if(cnt == 2)    Mission = 1;
        else if(cnt == 4)   Mission = 5;
        else if(cnt == 6)   Mission = 3;
        else Mission = 0;
    }
    else if(MODE == 3)//narae
    {
        if(cnt == 2)
        {
            if(!isPark)
                Mission = 6; //PAKRING
                
            else
                Mission = 0;
        }   
        else if(cnt == 4)
        {
            Mission = 1;
        }
        else    Mission = 0;

    }
    else if(MODE ==4)
    {
        // if(cnt == 1)    Mission = 5;
        if(cnt == 2 || cnt==99)   Mission = 3;
        else Mission = 0;
    }
    mission_.data = Mission;
    missionPub.publish(mission_);
    ros::spinOnce();
}

void MissionControl()
{
    switch (Mission) 
    {
    /*
    * MISSION => 0:x, 1:신호등직진, 2:신호등 좌회전, 3:정적소형, 4:정적대형, 5:동적, 6:주차, 7:배달
    * YOLO => 0:GreenLight, 1:RedLight, 2:GreenLeft, 3:Stopsign, 4:A1, 5:A2, 6:A3, 7:B1, 8:B2, 9:B3
    */
    
    case 0:
    /*
    * 미션 없음
    * gps를 사용하여 정상 주행
    */

        flag = true;
        break;
    case 1:
    /*
    * 신호등 직진 미션
    * 정지선이 일정 거리 아래로 감지되면서(getStopDistance), 신호등이 빨간불일경우(getVisionClass(2)) Estop()
    */
        //if (inputData->getStopDistance() < 5.0 && inputData->getVisionClass(2)) 
        if(inputData->getStopDistance() < 6.0 && inputData->getVisionClass(0)) 
            EStop();
        else
        {
            SetDrive();
            flag = true;
        }
        break;
    case 2:
    /*
    * 신호등 좌회전 미션
    * 정지선이 일정 거리 아래로 감지되면서(getStopDistance), 신호등이 빨간불일경우(getVisionClass(2)) Estop()
    */
        if (inputData->getStopDistance() < 6.0 && (inputData->getVisionClass(0) || inputData->getVisionClass(2))) 
            EStop();
        else
        {
            SetDrive();
            flag = true;
        }
        break;
    case 3:
    /*
    * 정적 소형 미션
    * lidarFlag가 0일경우 감지시작 안함, 1일경우 감지된 상태, 2일경우 모두 완료된 상태
    * 1일 경우 스티어값을 전달받아 시리얼로전달함
    */

        if(lidarFlag == 1)
        {
            control->SetWheelSpeed(8.0);

            flag = false;
        }
        else
        {
            SetDrive();
            flag = true;
        }
        break;
    case 4: 
    /*
    * 정적 대형 미션
    * 라이다로 부터 장애물이 감지 되면서(getBigObject)  지금까지 대형 장애물이 감지되지 않은 경우(checkBigObject)
    * cnt를 cntTemp에 임시 저장후 cnt를 0번으로 바꿈(0번 텍스트파일은 대형장애물 차선 임시파일)
    * 이후 다시한번 BigObject가 감지되었을 경우 cnt를 원래대로 되돌림
    */
        if(inputData->getBigObject() && !checkBigObject)
        {
            checkBigObject = true;
            if(cntTemp == -1){
                cntTemp = cnt;
                cnt = 99;
            }
            else
            {
                cnt = cntTemp;
            }
        }
        if(checkBigObject && !(inputData->getBigObject()))
            checkBigObject = false;
        
        flag = true;
        
        break;
    case 5:
    /*
    * 동적 미션encoder", 1000);
	ros::Publisher speed_pub = n.advertise<std_msgs::Float32>("speed", 1000);
	ros::Publisher steer_pub = n.advertise<std_msgs::Float32>("steer", 1000);
	ros::Publisher gear_pub = n.advertise<std_msgs::Float32>("gearicObject) Estop
    * 동적 장애물이 사라질 때까지 유지후, 다시 gps로 주행
    */
        if(inputData->getDynamicObject())
            EStop();
        else
        {
            SetDrive();
            flag = true;
        }
        break;
    case 6: //주차
    /*
    * 주차 미션
    * 플래닝에서는 미션 퍼블리시(Mission 6번), flag관리(직접 제어하므로 flag = false)만 하고 관리안함
    * 이후 엔코더까지 완료 되는 경우(isPark)
    * 주차미션 종료(setMission 에서 관리)
    */
        if(canPark)
            flag = false;
        else
            flag = true;
        break;
    case 7:
    /*
    * 배달 미션
    * 추가 예정
    */
    // SetDrive();
        if(cntTemp_d == -1)
        {
            cntTemp_d = cnt;
            cnt = 98;
        }
        if(timeStart)
        {
            ros::Duration(5.0).sleep();
            SetDrive();
            flag = true;
            timeStart = false;
            cnt = cntTemp_d;
            cntTemp_d = -2;
            lidarStopA = false;
        }
        if(lidarStopA)
        {
            EStop();
            if(!timeStart)
            {
                deli_time = clock();
                timeStart = true;
            }
        }
       // ROS_INFO("clock :%d\n",clock());
        //cout<<"asdfasdfasdf: "<<clock()<<endl;
       /* if(timeStart && clock() - deli_time > 500000)
        {
            SetDrive();
            flag = true;
            timeStart = false;
            cnt = cntTemp_d;
            cntTemp_d = -2;
            lidarStopA = false;
        }*/
        
        break;

    case 8:
        if(cntTemp_d == -2)
        {
            cntTemp_d = cnt;
            cnt = 97;
        }
        if(timeStart)
        {
            ros::Duration(5.0).sleep();
            SetDrive();
            flag = true;
            timeStart = false;
            cnt = cntTemp_d;
            cntTemp_d = -3;
            lidarStopA = false;
        }
        if(lidarStopA)
        {
            EStop();
            if(!timeStart)
            {
                deli_time = clock();
                timeStart = true;
            }
           
        }
        /*
         if(timeStart && clock() - deli_time >500000)
            {
                SetDrive();
                flag = true;
                timeStart = false;
                cnt = cntTemp_d;
                cntTemp_d = -3;
                lidarStopA = false;
            }*/
        
        break;
        
    }
}

void printMission()
{   
    if(!flag)
        printf(" Speed : %lf \n Angle : %lf \n Brake : %lf \n", control->GetWheelSpeed(), control->GetWheelAngle(), control->GetBrake());
    else
        printf(" Speed : - \n Angle : - \n Brake : - \n");
    printf("test : %d, Flag --> %s\n", test, flag ? "True" : "False");
    printf("BigObject : %s, checkBigObject : %s, cntTemp  : %d\n", inputData->getBigObject()    ?"True":"False", checkBigObject?"True":"False", cntTemp);
    inputData->printData();
    printf("Mission : ");
    switch (Mission)
    {
    case 1:
        printf("신호등 직진\n");
        break;
    case 2:
        printf("신호등 좌회전\n");
        break;
    case 3:
        printf("정적장애물 소형\n");
        break;
    case 4:
        printf("정적장애물 대형\n");
        break;
    case 5:
        printf("동적장애물\n");
        break;
    case 6:
        printf("주차\n");
        break;
    case 7:
        printf("배달 받기\n");
        break;
    case 8:
        printf("배달 하기\n");
        break;
    case 10:
        printf("주차 예비\n");
        break;
    default:
        printf("미션 없음\n");
        break;
    }
    printf("--------------\n");
}

void EStop()
{
    /*
    * Estop()
    * flag를 false로 바꾸어 gps로 이동하는것 방지
    * 속도 0, 조향 0, 브레이크 최대
    */
    flag = false;
    control->SetWheelSpeed(0.0);
    control->SetWheelAngle(0.0);
    control->SetBrake(180.0);
}
void SetDrive()
{
    /*
    * SetDrive()
    * 속도, 브레이크 복구
    */
   flag = false;
   control->SetWheelSpeed(6.0);
   control->SetWheelAngle(0.0);
   control->SetBrake(0.0);
}