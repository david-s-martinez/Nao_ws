#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from naoqi_bridge_msgs.msg import SpeechWithFeedbackActionGoal
from naoqi_bridge_msgs.msg import SetSpeechVocabularyActionGoal
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import HeadTouch
# from geometry_msgs.msg import Pose2D
# from sensor_msgs.msg import JointState
# from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
# from naoqi_bridge_msgs.msg import HandTouch
# from naoqi_bridge_msgs.msg import Bumper
# from std_msgs.msg import ColorRGBA
# from naoqi_bridge_msgs.msg import BlinkActionGoal

import random
import time
import threading

stop_thread = False

def spin_thread():
    global stop_thread
    while not stop_thread:
        rospy.spin_once()

class NAOTalk:

    def __init__(self):

        rospy.init_node('nao_talk', anonymous=True)
        self.rate = rospy.Rate(1)
        self.speech_pub = rospy.Publisher("/speech_action/goal", SpeechWithFeedbackActionGoal, queue_size=10)
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
                "You got me this time!",
                "Well played! I'll get you in the next round.",
                "I'm letting you win, you know."
            ],
            'starter': [ 
                "Hello, let's play UNO! I hope you are ready for a challange",
                "Welcome to our UNO game. I am excited to see who will win",
                "Ready to start the game, Can someone shuffle the cards?"

            ],
            'last_card_action' : [
                "Last card cannot be an action card. Let me draw a card."
            ],
            'throw_card' : [
                "Hmm, Let me see which card to play",
                "I think this card will do nicely",
            ],


        }

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


class NaoListen:
    def __init__(self):
        rospy.init_node('nao_listen', anonymous=True)
        self.voc_params_pub = rospy.Publisher("/speech_vocabulary_action/goal", SetSpeechVocabularyActionGoal, queue_size=1)
        self.recog_start_srv = rospy.ServiceProxy("/start_recognition", Empty)
        self.recog_stop_srv = rospy.ServiceProxy("/stop_recognition", Empty)
        self.recog_sub = rospy.Subscriber("/word_recognized", String, self.speech_recognition_cb)
        self.recognized_words = []
    
    def set_vocabulary(self):
        vocab_msg = SetSpeechVocabularyActionGoal()
        # vocab_msg.goal.words.extend(["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"])  # Can even be defined in separate sets (another func) if necessary
        # vocab_msg.goal.words.extend(["red", "blue", "green", "yellow", "draw", "wild", "skip", "reverse"])                → another set would allow for non-overlapping sets
        vocab_msg.goal.words.extend(["Pull", "Done", "Win", "Yes", "No"]) 
        self.voc_params_pub.publish(vocab_msg)
        rospy.loginfo("Published new speech vocabulary.")

    def speech_recognition_cb(self, msg):
        for word, confidence in zip(msg.words, msg.confidence_values):
            self.recognized_words.append(word)
            rospy.loginfo(f"Recognized: {word} with confidence: {confidence}")

    def listen(self, ):
        rospy.loginfo("Vocab Storing")
        self.set_vocabulary()
        rospy.loginfo("Speech Recognition Starts")
        self.recog_start_srv.call()

        if len(self.recognized_words) != 0:
            rospy.loginfo("Speech Recognition Stops")
            self.recog_stop_srv.call()

            # if not self.recognized_words:
            #     words = SpeechWithFeedbackActionGoal()
            #     words.goal.say = "stop playing with my buttons"
            #     self.speech_pub.publish(words)
            # for recognized_word in self.recognized_words:
                # words = SpeechWithFeedbackActionGoal()
                # rospy.loginfo("Talking")
                # words.goal.say = recognized_word
            # self.speech_pub.publish(words)
            
            temp = self.recognized_words
            rospy.sleep(1.0)

            self.recognized_words = []
        
        return temp


color = ('RED','GREEN','BLUE','YELLOW')
rank = ('0','1','2','3','4','5','6','7','8','9','Skip','Reverse','Draw2','Draw4','Wild')
ctype = {'0':'number','1':'number','2':'number','3':'number','4':'number','5':'number','6':'number',
            '7':'number','8':'number','9':'number','Skip':'action','Reverse':'action','Draw2':'action',
            'Draw4':'action_nocolor','Wild':'action_nocolor'}

class Card:

    def __init__(self, color, rank):
        self.rank = rank
        if color in ('RED','GREEN','BLUE','YELLOW',''):
            if ctype[rank] == 'number': 
                self.color = color
                self.cardtype = 'number'
            elif ctype[rank] == 'action':
                self.color = color
                self.cardtype = 'action'
            else:
                self.color = None
                self.cardtype = 'action_nocolor'

    def __str__(self):
        if self.color == None:
            return self.rank
        else:
            return self.color + " " + self.rank

class NAOHand:

    def __init__(self):
        self.cards = []
        self.cardsstr = []
        self.number_cards = 0
        self.action_cards = 0

    def add_card(self, card):
        self.cards.append(card)
        self.cardsstr.append(str(card))
        if card.cardtype == 'number':
            self.number_cards += 1
        else:
            self.action_cards += 1

    def remove_card(self, place):
        self.cardsstr.pop(place - 1)
        return self.cards.pop(place - 1)

    def cards_in_hand(self):
        for i in range(len(self.cardsstr)):
            print(f' {i + 1}.{self.cardsstr[i]}')

    def single_card(self, place):
        return self.cards[place - 1]

    def no_of_cards(self):
        return len(self.cards)

#Funciton to randomly select who starts first
def choose_first():
    if random.randint(0,1)==0:
        return 'Player'
    else:
        return 'NAO'


#Function to check if the card thrown by Player/NAO is a valid card by comparing it with the top card
def single_card_check(top_card,card):
    if card.color==top_card.color or top_card.rank==card.rank or card.cardtype=='action_nocolor':
        return True
    else:
        return False


#FOR NAO ONLY
#To check if NAO has any valid card to throw 
def full_hand_check(hand,top_card):
    for c in hand.cards:
        if c.color==top_card.color or c.rank == top_card.rank or c.cardtype=='action_nocolor':
            return hand.remove_card(hand.cardsstr.index(str(c))+1)
    else:
        return 'no card'


#Function to check if NAO wins
def win_check(hand):
    if len(hand.cards)==0:
        return True
    else:
        return False


#Function to check if last card is an action card (GAME MUST END WITH A NUMBER CARD)
def last_card_check(hand):
    for c in hand.cards:
        if c.cardtype!='number':
            return True
        else:
            return False

def deal_card():
    ranc = input("\nWhat is my card number or action? (#/Skip/Reverse/Draw2/Draw4/Wild)  ")
    if ranc not in ("Draw4","Wild"):
        colour = input("\nWhat is my card color? (Color)  ")
        if colour != colour.upper():
            colour = colour.upper()
    else: colour = ""

    return Card(colour,ranc)

def topcard():
    ranc = input("\nWhat is my card number or action? (#/Skip/Reverse/Draw2/Draw4/Wild)  ")
    if ranc not in ("Draw4","Wild"):
        colour = input("\nWhat is my card color? (Color)  ")
        if colour != colour.upper():
            colour = colour.upper()
    else: colour = ""

    return Card(colour,ranc)


def main():
    nao_hand = NAOHand()
    top_card = None
    nao_talk = NAOTalk()
    nao_listen = NaoListen()
    
    # rate_sleep = rospy.Rate(20)
    # while not rospy.is_shutdown():
    #     rate_sleep.sleep()

    while True:
        rospy.sleep(10)
        # NAO SPEECH 
        event_type = 'starter'
        nao_talk.run_speech(event_type)

        ###### Computer vision system integration here
        ### Let's say there is a function for detecting the cards
        #top_card = get_top_card()     → CompVision Module would replace topcard() func  
        #new_cards = get_new_cards()    → CompVision Module would replace deal_card() func
        ############
        #for card in new_cards:
            #nao_hand.add_card(card)    → already implemented

        print('Welcome to UNO! Finish your cards first to win')

        nao_hand = NAOHand()
        for i in range(6):        
            nao_hand.add_card(deal_card())
        print("NAO's cards are:  ")
        nao_hand.cards_in_hand()

        player_no_cards = 6

        time.sleep(1)
        playing = True

        turn = choose_first()
        print(turn + ' will go first')
        #NAO SPEECH 
        say = str(str(turn) + "will go first")
        nao_talk.run_given_speech(say,3)

        if turn == 'NAO':
            top_card = topcard()

        while playing:
            
            if turn == 'Player':
                #NAO SPEECH 
                say = str("Your turn")
                nao_talk.run_given_speech(say,3)

                throw_pulled = ""
                # NAO LISTEN
                choice = input("\nHit or Pull? (h/p): ")  # Replace by Speech recognition module
                # choice = nao_listen.listen()

                if choice == 'Pull':
                    player_no_cards += 1
                    # NAO LISTEN
                    throw_card = input("\nAre you throwing the new card? (y/n): ") # Replace by Speech recognition module
                    # throw_card = nao_listen.listen()
                    turn = 'NAO'

                if choice == 'Done' or throw_pulled == 'Yes':
                    player_no_cards -= 1
                    top_card = topcard()
                    if top_card.cardtype == 'number':
                        turn = 'NAO'
                    else:
                        if top_card.rank == 'Skip':
                            turn = 'Player'
                        elif top_card.rank == 'Reverse':
                            turn = 'Player'
                        elif top_card.rank == 'Draw2':
                            nao_hand.add_card(deal_card())
                            nao_hand.add_card(deal_card())
                            turn = 'Player'
                        elif top_card.rank == 'Draw4':
                            for i in range(4):
                                nao_hand.add_card(deal_card())
                            draw4color = input('Change color to (enter in caps): ')
                            if draw4color != draw4color.upper():
                                draw4color = draw4color.upper()
                            top_card.color = draw4color
                            turn = 'Player'
                        elif top_card.rank == 'Wild':
                            wildcolor = input('Change color to (enter in caps): ')
                            if wildcolor != wildcolor.upper():
                                wildcolor = wildcolor.upper()
                            top_card.color = wildcolor
                            turn = 'NAO'
                
                if player_no_cards == 0 :
                    print('\nPLAYER WON!!')
                    # NAO SPEECH
                    event_type = 'losing'
                    nao_talk.run_speech(event_type)
                    playing = False
                    break
            
            if turn == 'NAO':
                if nao_hand.no_of_cards() == 1:
                    if last_card_check(nao_hand):
                        time.sleep(1)
                        print('Adding a card to NAO hand')
                        nao_hand.add_card(deal_card)
                temp_card = full_hand_check(nao_hand, top_card)
                time.sleep(1)
                if temp_card != 'no card':
                    print('\nNAO throws:{}'.format(temp_card))
                    time.sleep(1)
                    if temp_card.cardtype == 'number':
                        top_card = temp_card
                        #NAO Speech
                        say = str(str(top_card) + "is on the top." )
                        nao_talk.run_given_speech(say,5)
                        turn = 'Player'
                    else:
                        if temp_card.rank == 'Skip':
                            turn = 'NAO'
                            top_card = temp_card
                        elif temp_card.rank == 'Reverse':
                            turn = 'NAO'
                            top_card = temp_card
                        elif temp_card.rank == 'Draw2':
                            top_card = temp_card
                            turn = 'NAO'
                        elif temp_card.rank == 'Draw4':
                            top_card = temp_card
                            draw4color = nao_hand.cards[0].color
                            print('Color changes to', draw4color)
                            top_card.color = draw4color
                            turn = 'NAO'
                        elif temp_card.rank == 'Wild':
                            top_card = temp_card
                            wildcolor = nao_hand.cards[0].color
                            print("Color changes to", wildcolor)
                            top_card.color = wildcolor
                            turn = 'Player'
                else:
                    print('\nNAO wants to pull a card from deck')
                    time.sleep(1)
                    temp_card = deal_card()
                    # NAO SPEECH
                    say = str("I drew" + str(temp_card))
                    nao_talk.run_given_speech(say,3)
                    if single_card_check(top_card, temp_card):
                        print(f'NAO throws: {temp_card}')
                        time.sleep(1)
                        if temp_card.cardtype == 'number':
                            top_card = temp_card
                            turn = 'Player'
                        else:
                            if temp_card.rank == 'Skip':
                                turn = 'NAO'
                                top_card = temp_card
                            elif temp_card.rank == 'Reverse':
                                turn = 'NAO'
                                top_card = temp_card
                            elif temp_card.rank == 'Draw2':
                                player_no_cards += 2
                                top_card = temp_card
                                turn = 'NAO'
                            elif temp_card.rank == 'Draw4':
                                player_no_cards += 4
                                top_card = temp_card
                                draw4color = nao_hand.cards[0].color
                                print('Color changes to', draw4color)
                                top_card.color = draw4color
                                turn = 'NAO'
                            elif temp_card.rank == 'Wild':
                                top_card = temp_card
                                wildcolor = nao_hand.cards[0].color
                                # NAO SPEECH
                                say = str("I switch the color to" + str(wildcolor))
                                nao_talk.run_given_speech(say,5)
                                print('Color changes to', wildcolor)
                                top_card.color = wildcolor
                                turn = 'Player'
                    else:
                        print('NAO doesnt have a card')
                        # NAO SPEECH
                        say = str("I don't have a card")
                        nao_talk.run_given_speech(say,5)
                        time.sleep(1)
                        nao_hand.add_card(temp_card)
                        turn = 'Player'
                print('\nNAO has {} cards remaining'.format(nao_hand.no_of_cards()))
                # NAO SPEECH
                say = str("I have "+ str(nao_hand.no_of_cards()) + "cards left.")
                nao_talk.run_given_speech(say,5)
                time.sleep(1)
                if win_check(nao_hand):
                    print('\nNAO WON!!')
                    # NAO SPEECH
                    event_type = 'winning'
                    nao_talk.run_speech(event_type)
                    playing = False

        new_game = input('Would you like to play again? (y/n)')
        if new_game == 'y':
            continue
        else:
            print('\nThanks for playing!!')
            break





if __name__ == '__main__':
	main()
