/*
 * This source file was written to control the fully-actuated hexrotor platform
 * within the optitrack environment using rc_override and the APM flight stack.
 *
 * written by: Jameson Lee
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <keyboard/Key.h>
#include <math.h>
#include <vector>
#include <iostream>

#define RATE_LOOP 0.025			// timer callback loop rate
#define TAKEOFF_HEIGHT 1.0f		// Desired Take off height
#define LANDING_HEIGHT 0.0f		// Desired Landing height
#define GROUNDED_HEIGHT 0.3f    // Desired Landed Height
#define HOME_X 0.0f				// desired home x
#define HOME_Y 0.0f				// desired home y
#define HOME_Z 1.0f				// desired home z
#define HOME_YAW 0.0f			// desired home yaw

// Control Modes
#define TAKEOFF 1				// from current x,y,psi, take off to desired takeoff height
#define HOME 2					// travel to home position
#define CONTROLLED 3			// allow reference changes
#define LANDING 4				// from current x,y,psi, return to ground
#define LANDED 5

// data collection mode
#define OUTPUT_CONTROL 1		// accumulate PID and output control effort
#define STANDBY 2				// set reference to current location, 0 PID elements

// reference mode
#define RELATIVE 1				// edit reference from current x,y,psi
#define GLOBAL 2				// edit reference from home position

#define ARMING 1				// edit reference from current x,y,psi
#define DISSARMING 2			// edit reference from home position

// reference filter values for smooth reference transitions
#define takeoff_d1 1.0f
#define takeoff_d2 30.0f
#define landing_d1 1.0f
#define landing_d2 0.5f
#define control_d1 1.0f
#define control_d2 0.25f

// PID gains for throttle edits
#define kp_th 0.15f//0.15f
#define ki_th 0.04f//0.04f
#define kd_th 0.3f//0.2f

#define kp_xy 0.75f//0.15f
#define ki_xy 0.07f//0.1f
#define kd_xy 1.5f//0.2f

#define kp_ya 0.05f//0.15f
#define ki_ya 0.0025f//0.04f
#define kd_ya 0.1f//0.2f


//////////////////////
// Keyboard Defines //
//////////////////////

#define Q_KEY 113
#define A_KEY 97
#define W_KEY 119
#define S_KEY 115
#define E_KEY 101
#define D_KEY 100
#define R_KEY 114
#define F_KEY 102
#define T_KEY 116
#define G_KEY 103
#define Y_KEY 121
#define H_KEY 104
#define ONE_KEY 49
#define TWO_KEY 50
#define THREE_KEY 51

class Hex{
	public:
		Hex();

	private:

        ros::NodeHandle nh;       			// ROS Node Handle

        ros::Subscriber keydown_sub;        // subscriber for keyboard downstroke
        ros::Subscriber keyup_sub;          // subscriber for keyboard upstroke
		ros::Subscriber opti_sub;           // subscriber for optitrack
		ros::Subscriber ref_sub;            // subscriber for reference setpoint calls
		ros::Subscriber mavros_sub;         // subscriber for mavros rc

		ros::Publisher rc_pub;              // Publishes rc override (1000-2000)
		ros::Publisher ref_pub;             // publish reference pose

		ros::Publisher rc0_pub;             // Publishes rc roll (1000-2000)
		ros::Publisher rc1_pub;             // Publishes rc pitch (1000-2000)
		ros::Publisher rc2_pub;             // Publishes rc throtte (1000-2000)
		ros::Publisher rc3_pub;             // Publishes yaw (1000-2000)
		ros::Publisher debug_pub;           // publish float 64 debug

        ros::Timer timer;                   // ROS timer object

        tf::TransformBroadcaster br;        // tf broadcaster object
        tf::TransformListener Li;           // tf Listener object

		ros::Time data_time_now;			// current time of optitrack pull callback
		ros::Time data_time_last;			// last time of optitrack pull callback
		ros::Time ref_time_now;				// current time of reference pull callback
		ros::Time ref_time_last;			// last time of reference pull callback

		mavros_msgs::OverrideRCIn rc_msg;	// rc override message
		geometry_msgs::PoseStamped ref_msg;	// rc override message

		std_msgs::UInt16 rc0_msg;			// matlab sucks so i needed to make individual messages for rc control of HIL
		std_msgs::UInt16 rc1_msg;			// matlab sucks so i needed to make individual messages for rc control of HIL
		std_msgs::UInt16 rc2_msg;			// matlab sucks so i needed to make individual messages for rc control of HIL
		std_msgs::UInt16 rc3_msg;			// matlab sucks so i needed to make individual messages for rc control of HIL
		std_msgs::Float64 debug_msg;		// debug float 64

		double pos[6];						// current position according to optitrack
		double ref[6];						// current reference sent to controller
		double ref_m1[6];					// last reference sent to controller
		double ref_in[6];					// current raw user setpoint

		double e[6];						// error vector
		double ei[6];						// integral error vector
		double ed[6];						// derivative error vector
		double e_last[6];					// last error vector

		double u[6];						// relative reference after PID to rc out
		double kp[6];
		double ki[6];
		double kd[6];
		double rc_out[8];					// rc override out converted from u
		double RC_MIN[8];					// vector with scalar rc max min values
		double RC_MAX[8];

		double data_dt;						// time between optitrack recieves
		double ref_dt;						// time between reference updates (should be timer loop rate)

		int mode;							// control mode
		int data_mode;						// specifies error accumilation or standby
		int ref_mode;						// TODO
		int mode_arm;

        //callback defines
        void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);
		void opti_cb(const geometry_msgs::PoseStampedConstPtr& pos_fb);
		void ref_cb(const geometry_msgs::PoseStampedConstPtr& ref_fb);
		void timer_cb(const ros::TimerEvent& event);
		void mavros_cb(const mavros_msgs::RCInConstPtr& rc_fb);

		double Quaternion2Roll(double qx, double qy, double qz, double qw);
		double Quaternion2Pitch(double qx, double qy, double qz, double qw);
		double Quaternion2Yaw(double qx, double qy, double qz, double qw);
};

Hex::Hex(){
    timer = nh.createTimer(ros::Duration(RATE_LOOP), &Hex::timer_cb, this);
	ref_sub = nh.subscribe<geometry_msgs::PoseStamped>("/hex_ref", 1, &Hex::ref_cb, this);
	opti_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn/hex/pose", 1, &Hex::opti_cb, this);
	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &Hex::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &Hex::keydown_cb, this);
	mavros_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &Hex::mavros_cb, this);

	rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
	ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/hex_ref", 1);
	rc0_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc0", 1);
	rc1_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc1", 1);
	rc2_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc2", 1);
	rc3_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc3", 1);
	debug_pub = nh.advertise<std_msgs::Float64>("/float_debug", 1);

	for(int i = 0; i < 6; i++){
		pos[i] = 0.0f;
		ref[i] = 0.0f;
		ref_m1[i] = 0.0f;
		ref_in[i] = 0.0f;
		e_last[i] = 0.0f;
		ei[i] = 0.0f;
	}
	ref[2] = TAKEOFF_HEIGHT;

	RC_MIN[0] = 1096.0f;
	RC_MIN[1] = 1100.0f;
	RC_MIN[2] = 1099.0f;
	RC_MIN[3] = 1105.0f;
	RC_MIN[4] = 1105.0f;
	RC_MIN[5] = 1105.0f;
	RC_MIN[6] = 1105.0f;
	RC_MIN[7] = 1105.0f;

	RC_MAX[0] = 1895.0f;
	RC_MAX[1] = 1900.0f;
	RC_MAX[2] = 1899.0f;
	RC_MAX[3] = 1905.0f;
	RC_MAX[4] = 1905.0f;
	RC_MAX[5] = 1905.0f;
	RC_MAX[6] = 1905.0f;
	RC_MAX[7] = 1905.0f;

	rc_out[0] = RC_MIN[0] + (RC_MAX[0] - RC_MIN[0])/2;
	rc_out[1] = RC_MIN[1] + (RC_MAX[1] - RC_MIN[1])/2;
	rc_out[2] = RC_MIN[2];
	rc_out[3] = RC_MIN[3] + (RC_MAX[3] - RC_MIN[3])/2;
	rc_out[4] = RC_MIN[4];
	rc_out[5] = RC_MIN[5];
	rc_out[6] = RC_MIN[6];
	rc_out[7] = RC_MIN[7];
	kp[0] = kp_xy;
	kp[1] = kp_xy;
	kp[2] = kp_th;
	kp[3] = 0.0f;
	kp[4] = 0.0f;
	kp[5] = kp_ya;

	ki[0] = ki_xy;
	ki[1] = ki_xy;
	ki[2] = ki_th;
	ki[3] = 0.0f;
	ki[4] = 0.0f;
	ki[5] = ki_ya;

	kd[0] = kd_xy;
	kd[1] = kd_xy;
	kd[2] = kd_th;
	kd[3] = 0.0f;
	kd[4] = 0.0f;
	kd[5] = kd_ya;

	data_time_last = ros::Time::now();
	ref_time_last = ros::Time::now();

	mode = LANDED;
	data_mode = STANDBY;
	ref_mode = GLOBAL;
}

void Hex::timer_cb(const ros::TimerEvent& event){
	ref_time_now = ros::Time::now();
	ref_dt = double((ref_time_now - ref_time_last).toSec());
	ref_time_last = ref_time_now;

	switch(mode){
		case TAKEOFF:
			data_mode = OUTPUT_CONTROL;
			ref_m1[2] = ref[2];
			ref[2] = (takeoff_d1/(takeoff_d1 + takeoff_d2*ref_dt))*ref_m1[2] + (takeoff_d1*takeoff_d2*ref_dt/(takeoff_d1 + takeoff_d2*ref_dt))*(TAKEOFF_HEIGHT);
			break;
		case CONTROLLED:
			data_mode = OUTPUT_CONTROL;
			for(int i = 0; i < 6; i++){
				ref_m1[i] = ref[i];
				ref[i] = (control_d1/(control_d1 + control_d2*ref_dt))*ref_m1[i] + (control_d1*control_d2*ref_dt/(control_d1 + control_d2*ref_dt))*ref_in[i];
			}
			break;
		case LANDING:
			if(ref[2] < GROUNDED_HEIGHT) data_mode = STANDBY;
			else data_mode = OUTPUT_CONTROL;
			ref_m1[2] = ref[2];
			ref[2] = (landing_d1/(landing_d1 + landing_d2*ref_dt))*ref_m1[2] + (landing_d1*landing_d2*ref_dt/(landing_d1 + landing_d2*ref_dt))*(LANDING_HEIGHT);
			break;
		case LANDED:
			data_mode = STANDBY;
			break;
		default:
			data_mode = STANDBY;
			break;
	}

	double cos_y = cos(pos[5]);
	double sin_y = -sin(pos[5]);

	switch(data_mode){
		case OUTPUT_CONTROL:
			rc_out[0] = int(RC_MIN[0] + (RC_MAX[0] - RC_MIN[0]) * (1.0f - u[1] * cos_y - u[0] * sin_y) / 2.0f);
			rc_out[1] = int(RC_MIN[1] + (RC_MAX[1] - RC_MIN[1]) * (1.0f - u[1] * sin_y + u[0] * cos_y) / 2.0f);
			rc_out[2] = int(RC_MIN[2] + (RC_MAX[2] - RC_MIN[2]) * (u[2]));
			rc_out[3] = int(RC_MIN[5] + (RC_MAX[5] - RC_MIN[5]) * (1.0f - u[5]) / 2.0f);
			break;
		case STANDBY:
			rc_out[0] = RC_MIN[0] + (RC_MAX[0] - RC_MIN[0])/2;
			rc_out[1] = RC_MIN[1] + (RC_MAX[1] - RC_MIN[1])/2;
			rc_out[2] = RC_MIN[2];
			rc_out[3] = RC_MIN[5] + (RC_MAX[5] - RC_MIN[5])/2;
			//for(int i = 0; i < 5; i++){
				//rc_out[i] = rc_msg.CHAN_NOCHANGE;
			//}
			break;
		default:
			break;
	}

	for(int i = 0; i < 5; i++){
		rc_msg.channels[i] = rc_out[i];
	}
	if(rc_out[0] != rc_msg.CHAN_NOCHANGE) rc_pub.publish(rc_msg);
	rc0_msg.data = rc_out[0];
	rc1_msg.data = rc_out[1];
	rc2_msg.data = rc_out[2];
	rc3_msg.data = rc_out[3];
	rc0_pub.publish(rc0_msg);
	rc1_pub.publish(rc1_msg);
	rc2_pub.publish(rc2_msg);
	rc3_pub.publish(rc3_msg);

	//debug_msg.data = ref_in[5];
	//debug_pub.publish(debug_msg);
}

void Hex::opti_cb(const geometry_msgs::PoseStampedConstPtr& pos_fb){
	data_time_now = ros::Time::now();
	data_dt = double((data_time_now - data_time_last).toSec());
	data_time_last = data_time_now;

	double qx = pos_fb -> pose.orientation.x;
	double qy = pos_fb -> pose.orientation.y;
	double qz = pos_fb -> pose.orientation.z;
	double qw = pos_fb -> pose.orientation.w;

	pos[0] = pos_fb -> pose.position.x;
	pos[1] = pos_fb -> pose.position.y;
	pos[2] = pos_fb -> pose.position.z;
	pos[3] = Hex::Quaternion2Roll(qx,qy,qz,qw);
	pos[4] = Hex::Quaternion2Pitch(qx,qy,qz,qw);
	pos[5] = Hex::Quaternion2Yaw(qx,qy,qz,qw);

	switch(data_mode){
		case OUTPUT_CONTROL:
			for(int i = 0; i < 6; i++){
				e_last[i] = e[i];
				e[i] = ref[i] - pos[i];
				ed[i] = (e[i] - e_last[i])/data_dt;
				ei[i] = ei[i] + e[i]*data_dt;
				u[i] = kp[i]*e[i] + ki[i]*ei[i] + kd[i]*ed[i];
				if(u[i] > 1.0f) u[i] = 1.0f;
				if(u[i] < -1.0f) u[i] = -1.0f;
			}
			if(u[2] < 0.0f) u[2] = 0.0f;
			break;
		case STANDBY:
			for(int i = 0; i < 6; i++){
				ref[i] = pos[i];
				e_last[i] = 0.0f;
				e[i] = 0.0f;
				ed[i] = 0.0f;
				ei[i] = 0.0f;
				u[i] = 0.0f;
			}
			break;
		default:
			break;
	}
}

void Hex::ref_cb(const geometry_msgs::PoseStampedConstPtr& ref_fb){
	/*double qx = ref_fb -> pose.orientation.x;
	double qy = ref_fb -> pose.orientation.y;
	double qz = ref_fb -> pose.orientation.z;
	double qw = ref_fb -> pose.orientation.w;

	ref_in[0] = ref_fb -> pose.position.x;
	ref_in[1] = ref_fb -> pose.position.y;
	ref_in[2] = ref_fb -> pose.position.z;
	ref_in[3] = Hex::Quaternion2Roll(qx,qy,qz,qw);
	ref_in[4] = Hex::Quaternion2Pitch(qx,qy,qz,qw);
	ref_in[5] = Hex::Quaternion2Yaw(qx,qy,qz,qw);*/
}

void Hex::mavros_cb(const mavros_msgs::RCInConstPtr& rc_fb){
	rc0_msg.data = rc_fb -> channels[0];
	rc1_msg.data = rc_fb -> channels[1];
	rc2_msg.data = rc_fb -> channels[2];
	rc3_msg.data = rc_fb -> channels[3];

	rc0_pub.publish(rc0_msg);
	rc1_pub.publish(rc1_msg);
	rc2_pub.publish(rc2_msg);
	rc3_pub.publish(rc3_msg);
}

double Hex::Quaternion2Roll(double qx, double qy, double qz, double qw){
	double t0 = 2.0*(qw*qx + qy*qz);
	double t1 = 1.0 - 2.0*(qx*qx + qy*qy);
	return atan2(t0, t1);
}

double Hex::Quaternion2Pitch(double qx, double qy, double qz, double qw){
	double t2 = 2.0*(qw*qy - qz*qx);
	t2 = ((t2 > 1.0) ? 1.0 : t2);
	t2 = ((t2 < -1.0) ? -1.0 : t2);
	return asin(t2);
}

double Hex::Quaternion2Yaw(double qx, double qy, double qz, double qw){
	double t3 = 2.0*(qw*qz + qx*qy);
	double t4 = 1.0 - 2.0*(qy*qy + qz*qz);
	return atan2(t3, t4);
}

void Hex::keydown_cb(const keyboard::KeyConstPtr& keydown){
	int x = keydown -> code;
	switch(x){
		case Q_KEY:
			ref_msg.pose.position.x = 0.0f;
			ref_msg.pose.position.y = 0.0f;
			ref_msg.pose.position.z = 1.0f;

			ref_msg.pose.orientation.x = 0.0f;
			ref_msg.pose.orientation.y = 0.0f;
			ref_msg.pose.orientation.z = 0.0f;
			ref_msg.pose.orientation.w = 1.0f;
			ref_pub.publish(ref_msg);
			break;
		case A_KEY:
			ref_msg.pose.position.x = 0.5f;
			ref_msg.pose.position.y = 0.25f;
			ref_msg.pose.position.z = 1.0f;

			ref_msg.pose.orientation.x = 0.0f;
			ref_msg.pose.orientation.y = 0.0f;
			ref_msg.pose.orientation.z = 0.5f;
			ref_msg.pose.orientation.w = sqrt(3.0f)/2;
			ref_pub.publish(ref_msg);
			break;
		case W_KEY:
			rc_out[4] = RC_MIN[4];
			break;
		case S_KEY:
			rc_out[4] = RC_MAX[4];
			break;
		case E_KEY:
			mode_arm = ARMING;
			break;
		case D_KEY:
			mode_arm = DISSARMING;
			break;
		case R_KEY:

			break;
		case F_KEY:

			break;
		case T_KEY:

			break;
		case G_KEY:

			break;
		case Y_KEY:

			break;
		case H_KEY:
			break;
        case ONE_KEY:
			mode = TAKEOFF;
            break;
        case TWO_KEY:
			mode = LANDING;
            break;
        case THREE_KEY:
			mode = CONTROLLED;
            break;
	}
}

void Hex::keyup_cb(const keyboard::KeyConstPtr& keyup){
    int x = keyup -> code;
    switch(x){
        case Q_KEY:

            break;
        case A_KEY:

            break;
        case W_KEY:

            break;
        case S_KEY:

            break;
        case E_KEY:

            break;
        case D_KEY:

            break;
        case R_KEY:

            break;
        case F_KEY:

            break;
        case T_KEY:

            break;
        case G_KEY:

            break;
        case Y_KEY:

            break;
        case H_KEY:

            break;
    }
}


int main(int argc, char **argv){
	ros::init(argc, argv, "Hex");
	ROS_INFO_STREAM("Generate Hex Reference Generation Node!");
	Hex hex;

	while(ros::ok()){
		ros::spinOnce();
	}
}
