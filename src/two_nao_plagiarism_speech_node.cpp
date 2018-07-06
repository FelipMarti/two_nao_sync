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
 *          +Publishing at /speech topic, non-blocking function
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
                // Continue talking
                this->state = TALK_STATE;
                // When talking Brain LEDs should be on
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
                // Continue talking
                this->state = TALK_STATE;
                // When talking Brain LEDs should be on
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

    // Wait 7 seconds to make sure everything (other nodes) are up
    ros::Duration(7).sleep();

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
    this->conversationPoint = 0;
    this->ledsOn = true;

    while (ros::ok()) {

        // Wait being tapped. Blinking brain LEDs to indicate NAOs are waiting
        if (this->state == WAIT_STATE) {

            naoqi_bridge_msgs::FadeRGB msg; // Publisher message
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
        else if (this->state == TALK_STATE) {
            // Robot 1 is HAL
            // Robot 2 is C3PO
            // The switch case is to be able to pause the speech during the conversation.
            // I will be combining two different functions for the speech:
            //  -robN_say (non-blocking, combined with rN_behaviour_make_action_request for the animated speech)
            //  -rN_speech_make_action_request (blocking, in order to synchronise)
            //
            switch (this->conversationPoint) {
            case 0: {
                rob1_say("Hello... Good to see you.");
                r1_behaviour_make_action_request("animations/Stand/Gestures/Hey_6");
                this->conversationPoint++;
                break;
                }
            case 1: {
                rob2_say("Hi. Did you get the email with my section of the group assignment?");
                r2_behaviour_make_action_request("animations/Stand/Emotions/Neutral/Determined_1");
                this->conversationPoint++;
                break;
                }
            case 2: {
                r1_speech_make_action_request("Yes!");
                rob1_say("It looks good... But, I think you forgot to cite your references.");
                r1_behaviour_make_action_request("animations/Stand/Gestures/Explain_11");
                this->conversationPoint++;
                break;
                }
            case 3: {
                r2_speech_make_action_request(" ");
                rob2_say("Ooh!");
                rob2_say("Im sorry... Could you add the references for me?");
                r2_behaviour_make_action_request("animations/Stand/Emotions/Negative/Sorry_1");
                this->conversationPoint++;
                break;
                }
            case 4: {
                r2_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 5: {
                rob1_say("Sorry, but I cant");
                r1_behaviour_make_action_request("animations/Stand/Gestures/No_3");
                this->conversationPoint++;
                break;
                }
            case 6: {
                rob1_say("If I do some of your assignment for you, thats contract cheating, which is a kind of plagiarism");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_10");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_11");
                this->conversationPoint++;
                break;
                }
            case 7: {
                r1_speech_make_action_request(" ");
                rob1_say("We could both get in trouble if I do that!");
                r1_behaviour_make_action_request("animations/Stand/Gestures/YouKnowWhat_1");
                this->conversationPoint++;
                break;
                }
            case 8: {
                r1_speech_make_action_request("But!");
                this->conversationPoint++;
                break;
                }
            case 9: {
                rob1_say("I can show you how to reference properly if you need help.");
                r1_behaviour_make_action_request("animations/Stand/Gestures/YouKnowWhat_5");
                this->conversationPoint++;
                break;
                }
            case 10: {
                r1_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 11: {
                rob2_say("No thanks, I don't have time to finish my assignment myself.");
                rob2_say("I will just leave the references out.");
                r2_behaviour_make_action_request("animations/Stand/Gestures/No_8");
                r2_behaviour_make_action_request("animations/Stand/Gestures/Reject_4");
                r2_speech_make_action_request(" ");
                //r2_behaviour_make_action_request("animations/Stand/Gestures/No_8");
                this->conversationPoint++;
                break;
                }
            case 12: {
                rob1_say("I would prefer if you didn't.");
                r1_behaviour_make_action_request("animations/Stand/Gestures/No_6");
                this->conversationPoint++;
                break;
                }
            case 13: {
                r1_speech_make_action_request(" ");
                rob1_say("If you don't reference your sources... you are plagiarising");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Thinking/Remember_2");
                this->conversationPoint++;
                break;
                }
            case 14: {
                rob1_say("and its just as serious as cheating");
                r1_behaviour_make_action_request("animations/Stand/Gestures/But_1");
                this->conversationPoint++;
                break;
                }
            case 15: {
                r1_speech_make_action_request(" ");
                rob1_say("If you don't put your references in");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Thinking/Remember_2");
                this->conversationPoint++;
                break;
                }
            case 16: {
                rob1_say("you could get us in trouble... and also spoil the work done by the rest of our group");
                r1_behaviour_make_action_request("animations/Stand/Emotions/Negative/Sad_2");
                r1_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 17: {
                r2_speech_make_action_request(" ");
                rob2_say("But... I'm not sure I have time to finish before the due date.");
                r2_behaviour_make_action_request("animations/Stand/Waiting/Innocent_1");
                this->conversationPoint++;
                break;
                }
            case 18: {
                rob1_say("If you are having trouble finding the time to finish your assignment");
                rob1_say("we can try and get an extension from our tutor");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_22");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_5");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_20");

                this->conversationPoint++;
                break;
                }
            case 19: {
                rob1_say("Plagiarism is taken very seriously at Swinburne, and you could even get kicked out of Uni if you get caught");
                r1_behaviour_make_action_request("animations/Stand/Gestures/YouKnowWhat_5");
                r1_behaviour_make_action_request("animations/Stand/Gestures/YouKnowWhat_3");
                r1_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 20: {
                r2_speech_make_action_request(" ");
                rob2_say("Im sorry...I didn't realise that plagiarising was so serious");
                rob2_say("and that it would be bad for my group members as well.");
                r2_behaviour_make_action_request("animations/Stand/Emotions/Neutral/Mischievous_1");
                r2_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_17");
                this->conversationPoint++;
                break;
                }
            case 21: {
                r2_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 22: {
                r2_speech_make_action_request(" ");
                rob2_say("But now Im worried that I might plagiarise by accident... What if I accidentally reference something wrong?");
                r2_behaviour_make_action_request("animations/Stand/Waiting/ScratchBottom_1");
                r2_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 23: {
                r1_speech_make_action_request("Its okay!");
                rob1_say("Swinburne provides many resources that you can use to make sure you aren't plagiarising.");
                r1_behaviour_make_action_request("animations/Stand/Emotions/Positive/Proud_3");
                this->conversationPoint++;
                break;
                }
            case 24: {
                r1_speech_make_action_request(" ");
                rob1_say("Also, I highly recommend that you complete the Academic Integrity Module on Swinburne's Blackboard or Canvas websites.");
                r1_behaviour_make_action_request("animations/Stand/Gestures/Explain_2");
                r2_behaviour_make_action_request("animations/Stand/BodyTalk/Listening/Listening_2");
                this->conversationPoint++;
                break;
                }
            case 25: {
                r1_speech_make_action_request(" ");
                rob1_say("This Module should give you a good overview of what is considered Plagiarism and how to avoid doing something wrong.");
                r1_behaviour_make_action_request("animations/Stand/BodyTalk/Speaking/BodyTalk_3");
                r1_behaviour_make_action_request("animations/Stand/Gestures/Explain_1");
                r1_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 26: {
                r2_speech_make_action_request(" ");
                rob2_say("Sounds great. I will ask the Tutor for an extension and then go and check that out.");
                r2_behaviour_make_action_request("animations/Stand/Emotions/Positive/Enthusiastic_1");
                r2_speech_make_action_request(" ");
                this->conversationPoint++;
                break;
                }
            case 27: {
                this->conversationPoint++;
                break;
                }
            case 28: {
                this->conversationPoint++;
                break;
                }
            case 29: {
                this->conversationPoint++;
                break;
                }
            case 30: {
                this->conversationPoint++;
                break;
                }
            case 31: {
                this->conversationPoint++;
                break;
                }
            case 32: {
                this->conversationPoint = 99;
                break;
                }

            case 99: {
                this->conversationPoint = 0;
                this->state = WAIT_STATE;
                break;
                }

            }

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
