// must have 
#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/thread/thread.h"
#include "src/common/string_Iv2/string_Iv2.h"
#include <CppLinuxSerial/SerialPort.hpp>
//  
using namespace std;
// settime process
long double ts_process1=0.1; //time set for process1
long double ts_process2=0.1; //time set for process2
long double ts_process3=0.1; //time set for process3
long double ts_process4=0.1; //time set for process4
long double ts_process5=0.1; //time set for process5
static pthread_mutex_t process_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static pthread_t p_process1;
static pthread_t p_process2;
static pthread_t p_process3;
static pthread_t p_process4;
static pthread_t p_process5;
// uart port
vector<uint8_t>  data_tran_uart={};
vector<uint8_t>  data_receive_uart={};
string name_port="/dev/my_USB1";
using namespace mn::CppLinuxSerial;
SerialPort serialPort(name_port, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
int button[4];
int led[4];
//
float v_set,w_set;
float v_max=0.12,w_max=0.314;
float time_out_cmd;
float v_read,w_read;
float odom_x=0,odom_y=0,odom_theta=0;
// create function system
void lock();
void unlock();
void function1();
void function2();
void function3();
void function4();
void function5();
void time_now(string data);
void view_data(string name, std::vector<uint8_t> data){
    cout<<name;
    for(int i=0;i<data.size();i++){
        printf("0x%x ",data[i]);
    }
    cout<<endl;
}
// creat thread
void * process1(void * nothing){
	char name[]="Process A";
	process(name,ts_process1,0,function1);
    void *value_return;
    return value_return;
}
void * process2(void * nothing){
	char name[]="Process B";
	process(name,ts_process2,1,function2);
    void *value_return;
    return value_return;
}
void * process3(void * nothing){
	char name[]="Process C";
	process(name,ts_process3,2,function3);
    void *value_return;
    return value_return;
}
void * process4(void * nothing){
	char name[]="Process D";
	process(name,ts_process4,3,function4);
    void *value_return;
    return value_return;
}
void * process5(void * nothing){
	char name[]="Process E";
	process(name,ts_process5,4,function5);
    void *value_return;
    return value_return;
}
//
void cmd_velf(const geometry_msgs::Twist& msg){
    lock();
        v_set=msg.linear.x;
        w_set=msg.angular.z;
        time_out_cmd=0;
        if(fabs(v_set)>=v_max) v_set=fabs(v_set)/v_set*v_max;
        if(fabs(w_set)>=w_max) w_set=fabs(w_set)/w_set*w_max;
    unlock();
}
void set_ledf(const std_msgs::String& msg){
    lock();
        cout<<"update led"<<endl;
        cout<<msg.data[0]<<endl;
        cout<<msg.data[1]<<endl;
        led[stoi(to_string(msg.data[0]))-48]=stoi(to_string(msg.data[1]))-48;
        // led[0]=1;
        // led[1]=1;

    unlock();
}
//
void pub_odom_wheel(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<nav_msgs::Odometry>("/odom",1);
    static float creat_fun=0;
        if(creat_fun==1)
        {
            static nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id="odom";
            odom.child_frame_id="base_link";
            odom_x=odom_x+v_read*cos(odom_theta)*0.1;
            odom_y=odom_y+v_read*sin(odom_theta)*0.1;
            odom_theta=odom_theta+w_read*0.1;
            //
            odom.pose.pose.position.x=odom_x;
            odom.pose.pose.position.y=odom_y;
            odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(odom_theta);
            odom.twist.twist.linear.x = v_read;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = w_read;
            //
            pub.publish(odom);  
        } else creat_fun=1;
}
void pub_button(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::UInt8MultiArray>("/button",1);
    static float creat_fun=0;
        if(creat_fun==1)
        {
            static std_msgs::UInt8MultiArray data;
            data.data.resize(4);
            for (int i = 0; i < 4; i++)
            {
                 data.data[i]=button[i];
            }
            pub.publish(data);  
        } else creat_fun=1;
}
int main(int argc, char** argv){
    // creat thread
    int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    // res=pthread_create(&p_process2,NULL,process2,NULL);
    // res=pthread_create(&p_process3,NULL,process3,NULL);
    // res=pthread_create(&p_process4,NULL,process4,NULL);
    // res=pthread_create(&p_process5,NULL,process5,NULL);
    //
    serialPort.SetTimeout(5);
    try{
        serialPort.Open();
    }
    catch(const std::exception& e){

    }
    //
    data_tran_uart.resize(16);
    data_tran_uart[0]=0xff;
    data_tran_uart[15]=0xff;
    //
    data_receive_uart.resize(16);
    //
    ros::init(argc, argv, "agv_core");
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20,n21,n22,n23;
    ros::Subscriber sub1 = n1.subscribe("/cmd_vel", 1, cmd_velf);
    ros::Subscriber sub2 = n2.subscribe("/set_led", 1, set_ledf);
    ros::spin();
    return 0;
}
//
void lock(){
    pthread_mutex_lock(&process_mutex);
}
void unlock(){
    pthread_mutex_unlock(&process_mutex);
}
void time_now(string name){
    lock();
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        cout<<name+"|Time:"<<std::fixed << std::setprecision(5)<<((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9)<<endl;
    unlock();
}
void function1(){
    lock();
        //
        time_out_cmd+=(float)ts_process1;
        if(time_out_cmd>=1.5){
            v_set=0;
            w_set=0;
        }
        // convert data to adruino
        data_tran_uart[1]=(uint8_t)(fabs(v_set)*100);
        if(v_set >0 )  data_tran_uart[2]=1;
        else data_tran_uart[2]=0;
        data_tran_uart[3]=(uint8_t)(fabs(w_set)*100);
        if(w_set >0 )  data_tran_uart[4]=1;
        else data_tran_uart[4]=0;
        //
        data_tran_uart[5]=led[0];
        data_tran_uart[6]=led[1];
        data_tran_uart[7]=led[2];
        data_tran_uart[8]=led[3];
        //
        view_data("pi said: \t",data_tran_uart);
        serialPort.WriteBinary(data_tran_uart);
    unlock();
        usleep(20000);
    lock();
        serialPort.ReadBinary(data_receive_uart);
        view_data("adruino said: \t",data_receive_uart);
        //
        v_read=(float)(data_receive_uart[3])/100;
        if(data_receive_uart[7]==0) v_read=-v_read; 
        //
        w_read=(float)(data_receive_uart[4])/100;
        if(data_receive_uart[8]==0) w_read=-w_read; 
        button[0]=data_receive_uart[9];
        button[1]=data_receive_uart[10];
        button[2]=data_receive_uart[11];
        button[3]=data_receive_uart[12];
        //
        pub_odom_wheel();
        pub_button();
    unlock();
}
void function2(){
        lock();
            printf("Action function 2\n"); 
            time_now("At:");
        unlock();
}
void function3(){
        lock();
            printf("Action function 3\n"); 
            time_now("At:");
        unlock();
      
}
void function4(){
        lock();
            printf("Action function 4\n"); 
            time_now("At:");
        unlock();
}
void function5(){
    lock();
        printf("Action function 5\n"); 
        time_now("At:");
    unlock();
}
