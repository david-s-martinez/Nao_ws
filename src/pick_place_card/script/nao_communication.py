#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from naoqi_bridge_msgs.msg import HeadTouch,SetSpeechVocabularyActionGoal, WordRecognized, SpeechWithFeedbackActionGoal, Bumper
from std_srvs.srv import Empty
from collections import Counter
from naoqi import ALProxy

import random
import time
import threading

class NAOCommunicate:

    def __init__(self):

        rospy.init_node('nao_talk', anonymous=True)
        self.rate = rospy.Rate(1)
        self.speech_pub = rospy.Publisher("/speech_action/goal", SpeechWithFeedbackActionGoal, queue_size=10)
        self.recog_start_srv = rospy.ServiceProxy('/start_recognition', Empty)
        self.recog_stop_srv = rospy.ServiceProxy("/stop_recognition", Empty)
        self.recog_sub = rospy.Subscriber('/word_recognized', WordRecognized, self.speech_recognition_cb, queue_size=10)
        self.vocab_pub = rospy.Publisher('/speech_vocabulary_action/goal',SetSpeechVocabularyActionGoal, queue_size=10)
        self.head = HeadTouch(0,0)
        self.head_sub = rospy.Subscriber("/tactile_touch", HeadTouch, self.headtouch_callback)
        self.bumper_sub = rospy.Subscriber("/bumper", Bumper, self.bumper_callback)
        self.bumper = None

        self.talk_phrases = {
            'draw_cards': [
                "Looks like you've got an armful there!",
                "I need to draw a card, none of mine can be played",
                "Drawing cards, are we? I hope you find what you're looking for!"
            ],
            'uno_call': [
                "UNO! I'm just one card away from victory!",
                "Just between us, I've always been good at this part.",
                "UNO! Did I just say random that out loud?"
            ],
            'winning': [
                "And that's how it's done!",
                "Victory is mine! Better luck next time.",
                "I guess I'm just programmed to win."
            ],                                                                                             
            'losing': [
                "You got me this time!Util",
                "Well played! I'll get you in the next round.",
                "I'm letting you win, you know."
            ],
            'starter': [ 
                "HelloNAOTalk, let's play UNO! I hope you are ready for a challange",
                "Welcome to our UNO game. I am excited to see who will win",
                "Ready to start the game, Can someone shuffle the cards?"

            ],

            'last_card_action' : [
                "Last card cannot be an action card. Let me draw a card."
            ],

            'throw_card' : [
                "Hmm, Let me see which card to play",
                "I think this card will do nicely"
            ],

            'draw2_card' : ["Double trouble coming your way!",
                            "Looks like you're getting a couple more friends!",
                            "Hope you're in the mood for more cards!",
                            "Twice the cards, twice the fun, right?",
                            "Two for you! Don't say I never give you anything."

            ],
            'draw4_card' : [ "Four more cards! It's your lucky day!",
                            "Guess who's drawing four? Spoiler: It's you!",
                            "Here's a little something extra for you!",
                            "Four's a party, isn't it?",
                            "I hope you've got room for four more!"

            ],
            'skip_card' : ["Oops, looks like you're taking a little break!",
                           "Skip-a-dee-doo-dah, skip-a-dee-day, your turn is flying away!",
                           "Not so fast! You're on pause.",
                           "Hold on, we're skipping your turn!",
                            "Looks like it's not your turn after all!"
            ],
            'reverse_card' : [  "Let's turn things around!",
                                "Reverse! Time to change directions.",
                                "And now for a little twist!",
                                "Back it up, we're reversing!",
                                "Time to go backwards, just for a bit!"
            ],
            'wild_card' : [ "Wild card! I'm shaking things up!",
                            "Time to spice things up with a wild card!",
                            "Let's change the color, shall we?",
                            "I'm feeling wild, let's mix it up!",
                              "Wild times call for wild cards!"
            ]




        }
        # self.recog_stop_srv = rospy.ServiceProxy("/stop_recognition", Empty)
        self.recognized_words = []
    
    def call_start_recognition(self):
        rospy.wait_for_service('/start_recognition')  
        
        try:
            # Call the service with an empty request
            response = self.recog_start_srv()
            # Process the response (empty for this service)
            print("Starting recognition.")
            print(self.create_vocabulary())
            rospy.sleep(5)
        except rospy.ServiceException as e:
            print("Service start_recognition failed", str(e))

    def call_stop_recognition(self, verbose=True):
        rospy.wait_for_service('/stop_recognition')  
        try:
            # Call the service with an empty request
            response = self.recog_stop_srv()
            
            # Process the response (empty for this service)
            if verbose:
                print("Stopping recognition.")
        except rospy.ServiceException as e:
            print("Service stop_recognition failed", str(e))

    def speech_recognition_cb(self, msg):
        print(len(msg.words))
        for i in range(len(msg.words)):
            recognized_word = msg.words[i]
            confidence = msg.confidence_values[i]

            # Store the words
            self.recognized_words.append(recognized_word)
            print("Recognized: {} with confidence: {}".format(recognized_word, confidence))
            rospy.loginfo("Recognized: {} with confidence: {}".format(recognized_word, confidence))

    def create_vocabulary(self):
  
        vocab_msg = SetSpeechVocabularyActionGoal()
        self.vocab_pub.publish(vocab_msg)
        # Add words to your vocabulary
        vocab_msg.goal.words.append("hit")
        vocab_msg.goal.words.append("pull")
        vocab_msg.goal.words.append("yes")
        vocab_msg.goal.words.append("no")
        word_list = ['RED','GREEN','BLUE','YELLOW','0','1','2','3','4','5','6','7','8','9','Skip','Reverse','Draw2','Draw4','Wild']
        for i in word_list:
            vocab_msg.goal.words.append(i)

        print(vocab_msg.goal.words)
        #rank = ()
        # Publish the vocabulary
        rospy.sleep(1)
        self.vocab_pub.publish(vocab_msg)
        rospy.sleep(1)
        print("Published new speech vocabulary.")

        rospy.loginfo("Published new speech vocabulary.")

    def start_voice_recognition(self):

        continue_game =False
        last_word = None
        while continue_game is False:

            if self.head.button is 1 and self.head.stage is 1:

                print('started voice recog')
                self.create_vocabulary()
                self.call_start_recognition()

            if self.head.button is 3 and self.head.stage is 1:

                rospy.sleep(3)
                print('stopped voice recog')
                self.call_stop_recognition()

            if self.head.button is 2 and self.head.stage is 1:
                if self.recognized_words:
                    last_word = self.recognized_words[-1]
                    continue_game = True
            return last_word, continue_game
        rospy.sleep(5)

    def headtouch_callback(self, headtouch):
        self.head = headtouch

    def bumper_callback(self, bump):
        self.bump = bump
 
     
    def detect_message(self):

        continue_game = False
        last_word = None
        
        while continue_game == False:
        
            if self.head.button is 1 and self.head.state is 1:
                self.create_vocabulary()
                self.call_start_recognition()
            elif  self.head.button is 3 and self.head.state is 1:
                self.call_stop_recognition(verbose=True)
                
            elif self.head.button is 2 and self.head.state is 1:
                if self.recognized_words:
                    last_word = self.recognized_words[-1]
                    continue_game = True
                self.call_stop_recognition()
                break
            else:
                pass

        return last_word, continue_game


    def run_speech(self, event_type):

        say = random.choice(self.talk_phrases[event_type])
        self.run_given_speech(say,7)

    def run_given_speech(self, input_speech, delay=10):
            
            
            # Construct the speech goal message
            talk_msg = SpeechWithFeedbackActionGoal()
            talk_msg.goal_id.id = str(rospy.Time.now().to_sec())
            talk_msg.goal.say = input_speech
            
            # Publish the trash talk line
            self.speech_pub.publish(talk_msg)
            rospy.sleep(delay)



