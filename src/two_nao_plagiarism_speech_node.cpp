/**
 *      This node controls two NAO robots to do a simple task such as 
 *      talking about plagiarism... Exciting!...
 *
 *      This code is based on the two nao robots counting together.
 *      ROS stuff coded in this code are the following:
 *
 *      -Subscribers to the tactile head interface (nao_apps) for R1 and R2
 *      -Publishers to the brain LEDs (nao_apps) for R1 and R2
 *      -Clients to Enable autonomous life mode (nao_apps) for R1 and R2
 *      -Clients to Disable autonomous life mode (nao_apps) for R1 and R2
 *      -Two ways to make the robots R1 and R2 talk:
 *          +Publishing at speech, non-blocking function
 *          +Actionlib (nao_apps), blocking function
 *      -Acionlib client to run behaviours (nao_apps) for R1 and R2
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2018 Felip Marti Carrillo  
 */


#include "two_nao_plagiarism_speech_node.h"


/**
 *  TwoNaoPlagiarismSpeech Constructor 
 */
TwoNaoPlagiarismSpeech::TwoNaoPlagiarismSpeech (void) :
    r1BehaviourActionClient("r1_run_behavior", true),
    r2BehaviourActionClient("r2_run_behavior", true),
    r1SpeechActionClient("r1_speech_action", true),
    r2SpeechActionClient("r2_speech_action", true)
{

    // [init publishers]
    this->pubR1Led = this->n.advertise<naoqi_bridge_msgs::FadeRGB>("r1_leds", 1);
    this->pubR2Led = this->n.advertise<naoqi_bridge_msgs::FadeRGB>("r2_leds", 1);
    this->pubR1Say = this->n.advertise<std_msgs::String>("r1_speech", 1);
    this->pubR2Say = this->n.advertise<std_msgs::String>("r2_speech", 1);
    
    // [init subscribers]
    this->subR1Head = this->n.subscribe("r1_tactile_head", 10,
                &TwoNaoPlagiarismSpeech::r1_head_callback, this);

    this->subR2Head = this->n.subscribe("r2_tactile_head", 10,
                &TwoNaoPlagiarismSpeech::r2_head_callback, this);


    // [init clients]
    r1AutLifeEnableClient = n.serviceClient<std_srvs::Empty>("r1_alife_enabled");
    r2AutLifeEnableClient = n.serviceClient<std_srvs::Empty>("r2_alife_enabled");
    r1AutLifeDisableClient = n.serviceClient<std_srvs::Empty>("r1_alife_disabled");
    r2AutLifeDisableClient = n.serviceClient<std_srvs::Empty>("r2_alife_disabled");

}


/**
 *  TwoNaoPlagiarismSpeech Destructor 
 */
TwoNaoPlagiarismSpeech::~TwoNaoPlagiarismSpeech (void) {
}


/**
 *  Robot 1 head tactile callback
 */
void TwoNaoPlagiarismSpeech::r1_head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg)
{

    if (msg->button == 1) {
        if (msg->state)
            ROS_INFO("[R1] Front Button has been Pressed");
        else
            ROS_INFO("[R1] Front Button has been Released");
    }
    if (msg->button == 2) {
        if (msg->state) {
            ROS_INFO("[R1] Middle Button has been Pressed");
        }
        else {
            ROS_INFO("[R1] Middle Button has been Released");
            if (this->state == WAIT_STATE) {
                // Continue counting from robot 1
                this->state = ROB1_STATE;
                // When counting Brain LEDs should be on
                naoqi_bridge_msgs::FadeRGB msg; //publisher message
                msg.led_name="BrainLeds";
                msg.fade_duration= ros::Duration(0.0);
                msg.color.r=1;
                msg.color.g=1;
                msg.color.b=1;
                pubR1Led.publish(msg);
                pubR2Led.publish(msg);
                ledsOn = true;
            }
            else {
                // Wait, stop counting
                this->state = WAIT_STATE;
            }
        }
    }
    if (msg->button == 3) {
        if (msg->state)
            ROS_INFO("[R1] Rear Button has been Pressed");
        else
            ROS_INFO("[R1] Rear Button has been Released");
    }

}


/**
 *  Robot 2 head tactile callback
 */
void TwoNaoPlagiarismSpeech::r2_head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg)
{

    if (msg->button == 1) {
        if (msg->state)
            ROS_INFO("[R2] Front Button has been Pressed");
        else
            ROS_INFO("[R2] Front Button has been Released");
    }
    if (msg->button == 2) {
        if (msg->state) {
            ROS_INFO("[R2] Middle Button has been Pressed");
        }
        else {
            ROS_INFO("[R2] Middle Button has been Released");
            if (this->state == WAIT_STATE) {
                // Continue counting from robot 2
                this->state = ROB2_STATE;
                // When counting Brain LEDs should be on
                naoqi_bridge_msgs::FadeRGB msg; //publisher message
                msg.led_name="BrainLeds";
                msg.fade_duration= ros::Duration(0.0);
                msg.color.r=1;
                msg.color.g=1;
                msg.color.b=1;
                pubR1Led.publish(msg);
                pubR2Led.publish(msg);
                ledsOn = true;
            }
            else {
                // Wait, stop counting
                this->state = WAIT_STATE;
            }
        }
    }
    if (msg->button == 3) {
        if (msg->state)
            ROS_INFO("[R2] Rear Button has been Pressed");
        else
            ROS_INFO("[R2] Rear Button has been Released");
    }

}


/**
 *  2 Functions to call the actionlib server for the behaviours, one for each robot
 */
void TwoNaoPlagiarismSpeech::r1_behaviour_make_action_request(std::string behaviour)
{
    ROS_INFO("[R1] Waiting for Behaviour action server to start");
    r1BehaviourActionClient.waitForServer(); //will wait for infinite time
    ROS_INFO("[R1] Behaviour action server started, sending goal");
    naoqi_bridge_msgs::RunBehaviorActionGoal behaviourGoal;
    behaviourGoal.goal.behavior = behaviour;
    r1BehaviourActionClient.sendGoal(behaviourGoal.goal);

    // Wait 30 seconds maximum
    bool finished_before_timeout = r1BehaviourActionClient.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        ROS_INFO("[R1] Behaviour action server has finished");
    }
    else {
        ROS_WARN("[R1] Behaviour action server hasn't finished on time...");
    }
}


void TwoNaoPlagiarismSpeech::r2_behaviour_make_action_request(std::string behaviour)
{
    ROS_INFO("[R2] Waiting for Behaviour action server to start");
    r2BehaviourActionClient.waitForServer(); //will wait for infinite time
    ROS_INFO("[R2] Behaviour action server started, sending goal");
    naoqi_bridge_msgs::RunBehaviorActionGoal behaviourGoal;
    behaviourGoal.goal.behavior = behaviour;
    r2BehaviourActionClient.sendGoal(behaviourGoal.goal);

    // Wait 30 seconds maximum
    bool finished_before_timeout = r2BehaviourActionClient.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        ROS_INFO("[R2] Behaviour action server has finished");
    }
    else {
        ROS_WARN("[R2] Behaviour action server hasn't finished on time...");
    }
}



/**
 *  2 Functions to call the actionlib server for the speech, one for each robot
 */
void TwoNaoPlagiarismSpeech::r1_speech_make_action_request(std::string text)
{
    ROS_INFO("[R1] Waiting for Speech action server to start");
    r1SpeechActionClient.waitForServer(); //will wait for infinite time
    ROS_INFO("[R1] Speech action server started, sending goal");
    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speechGoal;
    speechGoal.goal.say = text;
    r1SpeechActionClient.sendGoal(speechGoal.goal);

    // Wait 30 seconds maximum
    bool finished_before_timeout = r1SpeechActionClient.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        ROS_INFO("[R1] Speech action server has finished");
    }   
    else {
        ROS_WARN("[R1] Speech action server hasn't finished on time...");
    }   
}


void TwoNaoPlagiarismSpeech::r2_speech_make_action_request(std::string text)
{
    ROS_INFO("[R2] Waiting for Speech action server to start");
    r2SpeechActionClient.waitForServer(); //will wait for infinite time
    ROS_INFO("[R2] Speech action server started, sending goal");
    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speechGoal;
    speechGoal.goal.say = text;
    r2SpeechActionClient.sendGoal(speechGoal.goal);

    // Wait 30 seconds maximum
    bool finished_before_timeout = r2SpeechActionClient.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        ROS_INFO("[R2] Speech action server has finished");
    }   
    else {
        ROS_WARN("[R2] Speech action server hasn't finished on time...");
    }   
}



/**
 *  2 Functions to publish the speech, one for each robot
 */
void TwoNaoPlagiarismSpeech::rob1_say(std::string text)
{
    std_msgs::String msg;
    msg.data = text;
    pubR1Say.publish(msg);
}

void TwoNaoPlagiarismSpeech::rob2_say(std::string text)
{
    std_msgs::String msg;
    msg.data = text;
    pubR2Say.publish(msg);
}



/**
 *  The Main function
 */
int TwoNaoPlagiarismSpeech::Main () 
{

    // Main Loop
    ros::Rate loop_rate(2);  // Hz

    // Wait 5 seconds to make sure everything (other nodes) are up
    ros::Duration(5).sleep();

    // Wake up the robots
    // Robot 1
    if (r1AutLifeEnableClient.call(empty_msg)) {
        ROS_INFO("[R1] Autonomous life on");
    }
    else {
        ROS_ERROR("[R1] Problem when calling Alife service");
    }
    // Robot 2
    if (r2AutLifeEnableClient.call(empty_msg)) {
        ROS_INFO("[R2] Autonomous life on");
    }
    else {
        ROS_ERROR("[R2] Problem when calling Alife service");
    }

    // Init variables
    this->state = WAIT_STATE;
    this->ledsOn = true;

    // Some speech before the robots start
    r1_speech_make_action_request("Lets talk about plagiarism!");
    //TODO...

    while (ros::ok()) {

        // Wait being tapped, blinking brain LEDs to indicate NAOs are waiting
        if (this->state == WAIT_STATE) {

            naoqi_bridge_msgs::FadeRGB msg; //publisher message
            msg.led_name="BrainLeds";
            msg.fade_duration= ros::Duration(0.0);
            if (this->ledsOn) {
                // Turning off
                msg.color.r=0;
                msg.color.g=0;
                msg.color.b=0;
                ledsOn = false;
            }
            else {
                // Turning on
                msg.color.r=1;
                msg.color.g=1;
                msg.color.b=1;
                ledsOn = true;
            }
            // Publishing message for 2 robots
            pubR1Led.publish(msg);
            pubR2Led.publish(msg);
        }
        // Robot 1 counts
        else if (this->state == ROB1_STATE) {

            rob1_say("Bla bla bla...");
            r1_behaviour_make_action_request("animations/Stand/Gestures/CountOne_1");

            this->state = ROB2_STATE;
        }
        // Robot 2 counts
        else if (this->state == ROB2_STATE) {

            rob2_say("Yeah... nah...");
            r2_behaviour_make_action_request("animations/Stand/Gestures/CountOne_1");

            this->state = ROB1_STATE;
        }
        // You are doing something wrong mate...
        else {
            ROS_ERROR("Non contemplated state: %u", this->state);
        }

        // ROS stuff
        ros::spinOnce();
        loop_rate.sleep();

    }

    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "two_nao_sync");

    TwoNaoPlagiarismSpeech foo;
    return foo.Main();
}
