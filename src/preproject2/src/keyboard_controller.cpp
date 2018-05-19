#include <termios.h>
#include <signal.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

#define COMMAND_TIMEOUT_SEC 0.2

// at full joystick depression you'll go this fast
double max_speed = 2.00;
double max_turn = 60.0*M_PI/180.0;

class TBK_Node
{
private:
    ackermann_msgs::AckermannDriveStamped drive_msg_stamped;
    ros::NodeHandle n_;
    ros::Publisher pub_;

public:
    TBK_Node()
    {
        pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);

    }
    ~TBK_Node() { }
    void keyboardLoop();
    void stopRobot()
    {
        drive_msg_stamped.drive.speed = 0.0;
        drive_msg_stamped.drive.steering_angle = 0.0;
        pub_.publish(drive_msg_stamped);
    }
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    TBK_Node tbk;

    boost::thread t = boost::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void
TBK_Node::keyboardLoop()
{
    char keyboard_input;
    double speed = 1.0;
    double steering = 1.0;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("q/e : increase/decrease the speed of the car by 10%");
    puts("r : reset the speed of the car");
    puts("---------------------------");
    puts("Moving around:");
    puts("        w     ");
    puts("   a    s    d");
    puts("---------------------------");
    puts("w : move straight forward");
    puts("a : turn left");
    puts("s : move backward");
    puts("d : turn right");
    puts("anything else : stop the car");
    puts("---------------------------");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;
        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &keyboard_input, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
            continue;

		switch(keyboard_input){
			case KEYCODE_W:
				drive_msg_stamped.drive.speed = speed; 
				drive_msg_stamped.drive.steering_angle = 0.0;
				pub_.publish(drive_msg_stamped);
				break;
				
			case KEYCODE_A:
				drive_msg_stamped.drive.speed = speed; 
				drive_msg_stamped.drive.steering_angle = steering;
				pub_.publish(drive_msg_stamped);
				break;

			case KEYCODE_S:
				drive_msg_stamped.drive.speed = -speed; 
				drive_msg_stamped.drive.steering_angle = 0.0;
				pub_.publish(drive_msg_stamped);
				break;

			case KEYCODE_D:
				drive_msg_stamped.drive.speed = speed; 
				drive_msg_stamped.drive.steering_angle = -steering;
				pub_.publish(drive_msg_stamped);
				break;

			case KEYCODE_Q:
				speed *= 1.1;
				if (speed > max_speed)
					speed = max_speed;
				break;

			case KEYCODE_E:
				speed *= 0.9;
				break;

			case KEYCODE_R:
				speed = 1.0;
				break;

			default:
				drive_msg_stamped.drive.speed = 0.0; 
				drive_msg_stamped.drive.steering_angle = 0.0;
				pub_.publish(drive_msg_stamped);
		}

        // TODO
        // ASCII CODE of keyboard input is contained in variable c
        // refer above defined acronym for ASCII codes, write a code that publishing
        // adequate control of pioneer which coincides with keyboard input.
        //
        // 1. make control when specific key is pressed. (compare 'keyboard_input' variable with predefined KEYCODEs)
        // 2. publish control
        // if you follow instruction carefully, then this is so easy.

    }
}
