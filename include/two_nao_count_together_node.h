/**
 *      This node controls two NAO robots doing a simple task such as 
 *      counting together.
 *
 *      In this code I'm merging different codes that can be found here:
 *      https://github.com/FelipMarti/nao_examples
 *
 *      -Subscribers to the tactile head interface (nao_apps) for R1 and R2
 *      -Publishers to the brain LEDs (nao_apps) for R1 and R2
 *      -Clients to Enable autonomous life mode (nao_apps) for R1 and R2
 *      -Clients to Disable autonomous life mode (nao_apps) for R1 and R2
 *      -Two ways to make the robots R1 and R2 talk:
 *          +Publishing at /speech topic, non-blocking function
 *          +Actionlib (nao_apps), blocking function
 *      -Acionlib client to run behaviours (nao_apps) for R1 and R2
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2018 Felip Marti Carrillo  
 */


#ifndef _TWO_NAO_COUNTING_TOGETHER_HPP
#define _TWO_NAO_COUNTING_TOGETHER_HPP

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"

#include "naoqi_bridge_msgs/FadeRGB.h"
#include "naoqi_bridge_msgs/HeadTouch.h"
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackAction.h>
#include <naoqi_bridge_msgs/RunBehaviorAction.h>

#define WAIT_STATE 0
#define ROB1_STATE 1
#define ROB2_STATE 2

class TwoNaoCountTogether {


private:

    // ROS Stuff
    ros::NodeHandle n;

    // Robot Variables
    unsigned int state;
    unsigned int counter;
    bool ledsOn;

    // [publisher attributes]
    ros::Publisher pubR1Led;
    ros::Publisher pubR2Led;
    ros::Publisher pubR1Say;
    ros::Publisher pubR2Say;
    void rob1_say (std::string text);
    void rob2_say (std::string text);

    // [subscriber attributes]
    ros::Subscriber subR1Head;
    ros::Subscriber subR2Head;
    void r1_head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg);
    void r2_head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg);

    // [client attributes]
    ros::ServiceClient r1AutLifeEnableClient;
    ros::ServiceClient r1AutLifeDisableClient;
    ros::ServiceClient r2AutLifeEnableClient;
    ros::ServiceClient r2AutLifeDisableClient;
    std_srvs::Empty empty_msg;

    // [action client attributes]
    actionlib::SimpleActionClient<naoqi_bridge_msgs::SpeechWithFeedbackAction> r1SpeechActionClient;
    actionlib::SimpleActionClient<naoqi_bridge_msgs::SpeechWithFeedbackAction> r2SpeechActionClient;
    void r1_speech_make_action_request(std::string text);
    void r2_speech_make_action_request(std::string text);
    actionlib::SimpleActionClient<naoqi_bridge_msgs::RunBehaviorAction> r1BehaviourActionClient;
    actionlib::SimpleActionClient<naoqi_bridge_msgs::RunBehaviorAction> r2BehaviourActionClient;
    void r1_behaviour_make_action_request(std::string behaviour);
    void r2_behaviour_make_action_request(std::string behaviour);
    

public:

    /**
     *  TwoNaoCountTogether Constructor 
     */
    TwoNaoCountTogether (void);

    /**
     *  TwoNaoCountTogether Destructor 
     */
    ~TwoNaoCountTogether (void);

    int Main ();


};

#endif
