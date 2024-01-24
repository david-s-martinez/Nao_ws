#!/usr/bin/env python
import random
import time

from move_to_object import MovetoTarget
from naoqi_bridge_msgs.msg import SpeechWithFeedbackActionGoal,WordRecognized
import rospy
from nao_communication import NAOCommunicate
#from nao_vision import NAOVision
color = ('RED','GREEN','BLUE','YELLOW',)
rank = ('0','1','2','3','4','5','6','7','8','9','Skip','Reverse','Draw2','Draw4','Wild')
ctype = {'0':'number','1':'number','2':'number','3':'number','4':'number','5':'number','6':'number',
            '7':'number','8':'number','9':'number','Skip':'action','Reverse':'action','Draw2':'action',
            'Draw4':'action_nocolor','Wild':'action_nocolor'}


class Card:

    def __init__(self, color, rank):
        self.rank = rank
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
'''class Deck:
    def __init__(self):
        self.deck = []
        for clr in color:
            for ran in rank:
                if ctype[ran] != 'action_nocolor':
                    self.deck.append(Card(clr, ran))
                    self.deck.append(Card(clr, ran))
                else:
                    self.deck.append(Card(clr, ran))   

    def __str__(self):
        deck_comp = ''
        for card in self.deck:
            deck_comp += '\n' + card.__str__()
        return 'The deck has ' + deck_comp

    def shuffle(self):
        random.shuffle(self.deck)

    def deal(self):
        return self.deck.pop()'''

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
            print(' {}.{}'.format(i+1, self.cardsstr[i]))

    def single_card(self, place):
        return self.cards[place - 1]

    def no_of_cards(self):
        return len(self.cards)

#Funciton to randomly select who starts first
def choose_first():
    if random.randint(0,1)==0:
        return 'NAO'
    else:
        return 'NAO'


#Function to check if the card thrown by NAO/Player is a valid card by comparing it with the top card
def single_card_check(top_card,card):
    if card.color==top_card.color or top_card.rank==card.rank or card.cardtype=='action_nocolor':
        return True
    else:
        return False

#To check if NAO has any valid card to throw
def full_hand_check(hand,top_card):
    for c in hand.cards:
        if c.color==top_card.color or c.rank == top_card.rank or c.cardtype=='action_nocolor':
            return hand.remove_card(hand.cardsstr.index(str(c))+1)
    else:
        return 'no card'

#To check if NAO wins
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
    ranc = raw_input("\nWhat is my card number or action? (#/Skip/Reverse/Draw2/Draw4/Wild)  ")
    if ranc not in ("Draw4","Wild"):
        colour = raw_input("\nWhat is my card color? (Color)  ")
        if colour != colour.upper():
            colour = colour.upper()
    else: colour = ""

    return Card(colour,ranc)

def topcard():
    ranc = raw_input("\nWhat is the upper card number or action? (#/Skip/Reverse/Draw2/Draw4/Wild)  ")
    if ranc not in ("Draw4","Wild"):
        colour = raw_input("\nWhat is my card color? (Color)  ")
        if colour != colour.upper():
            colour = colour.upper()
    else: colour = ""

    return Card(colour,ranc)


def main():
    nao_hand = NAOHand()
    top_card = None
    nao_talk = NAOCommunicate()
    player_hand = NAOHand()
    #deck = Deck()
    #nao_vision = NAOVision()
    nao_move = MovetoTarget(needs_node=False)

    # rospy.spin()

    while True:

        #rospy.sleep(10)
        # NAO SPEECH 
        event_type = 'starter'
        nao_talk.run_speech(event_type)

        rospy.sleep(6)

        nao_talk.run_given_speech('Can you help me put the cards', 5)
        counter = 0 

        #############################################################################
        for counter_cards in range(7):

        # Computer Vision Algorithm 
            while nao_talk.bump.state == 0 :
                pass
            if nao_talk.bump.bumper == 0:
                rospy.sleep(2)
                print('Right foot pressed')
                #card = get_top_card()
           
                #nao_hand.add_card(card) # i.e. append
        print('Game Start')


        rospy.sleep(5)
        nao_move.move_head_Start()
        #Nao Say lets start the game 
        nao_talk.run_given_speech("Let's start!", 3)
        ###### Computer vision system integration here
        ### Let's say there is a function for detecting the cards
        #top_card = get_top_card()
        #new_cards = get_new_cards()
        ############
        #for card in new_cards:
            #nao_hand.add_card(card)
        

        #deck.shuffle()

        turn = choose_first()
        if turn == 'NAO':
            #NAO SPEECH 
            say = str( "I will go first")
            
        else:
             print(+ ' You go first')
        nao_talk.run_given_speech(say,3)



        top_card = topcard()
        if top_card.cardtype != 'number':
            while top_card.cardtype != 'number':
                top_card = topcard()
            

        print('\nStarting Card is: {}'.format(top_card))
        time.sleep(1)
        # NAO SPEECH 
        say = str("Starting Card is" + str(top_card))
        nao_talk.run_given_speech(say,3)
        playing = True

     

        


        while playing:

            if turn == 'Player':
                #NAO SPEECH 
                event_type = 'opponents_turn'
                nao_talk.run_given_speech(event_type)
                time.sleep(7)

                throw_pulled = ""
                # NAO LISTEN
                # choice = raw_input("\nHit or Pull? (h/p): ")  # Replace by Speech recognition module
                choice,_ = nao_talk.detect_message()
                if choice == 'pull':
                    player_no_cards += 1
                    # NAO LISTEN
                    throw_pulled = raw_input("\nAre you throwing the new card? (y/n): ") # Replace by Speech recognition module
                    turn = 'NAO'

                if choice == 'hit':
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
                            nao_move.move_RArm_withHead()
                            nao_hand.add_card(deal_card())
                            #detected_card = nao_vision.detect_card()
                            #nao_talk.start_voice_recognition()
                            #nao_hand.append(detected_card)
                            nao_move.move_RArm_withHead()
                            nao_hand.add_card(deal_card())
                            #detected_card = nao_vision.detect_card()
                            #nao_talk.start_voice_recognition()
                            #nao_hand.append(detected_card)
                            turn = 'Player'
                        elif top_card.rank == 'Draw4':
                            for i in range(4):
                                nao_move.move_RArm_withHead()
                                nao_hand.add_card(deal_card())
                                #detected_card = nao_vision.detect_card()
                                #nao_talk.start_voice_recognition()
                                #nao_hand.append(detected_card)
                            draw4color = raw_input('Change color to (enter in caps): ')
                            if draw4color != draw4color.upper():
                                draw4color = draw4color.upper()
                            top_card.color = draw4color
                            turn = 'Player'
                        elif top_card.rank == 'Wild':
                            wildcolor = raw_input('Change color to (enter in caps): ')
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


                # NAO Speech
                # event_type = 'opponents_turn'
                # nao_talk.run_given_speech(event_type)
                # time.sleep(7)
                # #top_card = nao_vision.detect_top_card()
                # if top_card.rank == 'Draw2':
                #     nao_move.move_RArm_withHead()
                #     #detected_card = nao_vision.detect_card()
                #     #nao_talk.start_voice_recognition()
                #     #nao_hand.append(detected_card)
                #     nao_move.move_RArm_withHead()
                #     # detected_card = nao_vision.detect_card()
                #     # nao_talk.start_voice_recognition()
                #     # nao_hand.append(detected_card)
                #     turn = 'Player'
                # if top_card.rank == 'Draw4':
                #     nao_move.move_RArm_withHead()
                #     # detected_card = nao_vision.detect_card()
                #     # nao_hand.append(detected_card)
                #     nao_move.move_RArm_withHead()
                #     # detected_card = nao_vision.detect_card()
                #     # nao_hand.append(detected_card)
                #     nao_move.move_RArm_withHead()
                #     # detected_card = nao_vision.detect_card()
                #     # nao_hand.append(detected_card)
                #     nao_move.move_RArm_withHead()
                #     # detected_card = nao_vision.detect_card()
                #     # nao_hand.append(detected_card)


                pass

            
            if turn == 'NAO':
                if nao_hand.no_of_cards() == 1:
                    if last_card_check(nao_hand):
                        time.sleep(1)
                        print('Adding a card to NAO hand')
                        # NAO Speech
                        say = str("My last card is Special card, Let me draw.")
                        nao_talk.run_given_speech(say, 5)
                        nao_move.move_RArm_withHead()
                        #detected_card = nao_vision.detect_card()
                        #nao_hand.add_card(detected_card)
                        nao_hand.add_card(deal_card())
                    else:
                        say = str("UNO")
                        nao_talk.run_given_speech(say, 5)

                temp_card = full_hand_check(nao_hand, top_card)
                if temp_card != 'no card':
                    # here nao will ask user to put desired card on shelf, then nao will pick from shelf to board.
                    nao_talk.run_given_speech("Please place {}".format(temp_card),5)
                    time.sleep(2)
                    nao_move.Left_Hand_Movement()
                    time.sleep(10)

                    if temp_card.cardtype == 'number':
                        top_card = temp_card
                        #NAO Speech
                        say = str(str(top_card) + "is on the top." )
                        nao_talk.run_given_speech(say,5)
                        turn = 'Player'
                    else:
                        if temp_card.rank == 'Skip':
                            # NAO SPEECH
                            event_type = 'skip_card'
                            nao_talk.run_speech(event_type)
                            turn = 'NAO'
                            top_card = temp_card
                        elif temp_card.rank == 'Reverse':
                            # NAO SPEECH
                            event_type = 'reverse_card'
                            nao_talk.run_speech(event_type)
                            turn = 'NAO'
                            top_card = temp_card
                        elif temp_card.rank == 'Draw2':
                            #player_hand.add_card(deck.deal())
                            #player_hand.add_card(deck.deal())
                            # NAO SPEECH
                            event_type = 'draw2_card'
                            nao_talk.run_speech(event_type)
                            top_card = temp_card
                            turn = 'NAO'
                        elif temp_card.rank == 'Draw4':
                            # NAO SPEECH
                            event_type = 'draw4_card'
                            nao_talk.run_speech(event_type)
                            # Assuming nao_hand.cards is a list of card objects with a 'color' attribute
                            colors = [card.color for card in nao_hand.cards]

                            # Use Counter to count occurrences of each color
                            color_counts = Counter(colors)
                            # Find the color with the maximum occurrence
                            most_common_color = color_counts.most_common(1)[0][0]
                            draw4color = most_common_color
                            #draw4color = nao_hand.cards[0].color
                            print('Color changes to', draw4color)
                            top_card.color = draw4color
                            top_card = temp_card
                            turn = 'NAO'
                            #for i in range(4):
                             #   player_hand.add_card(deck.deal())
                          
                        elif temp_card.rank == 'Wild':
                            # NAO SPEECH
                            event_type = 'wild_card'
                            nao_talk.run_speech(event_type)
                            top_card = temp_card
                             # Use Counter to count occurrences of each color
                            colors = [card.color for card in nao_hand.cards]
                            color_counts = Counter(colors)
                            # Find the color with the maximum occurrence
                            most_common_color = color_counts.most_common(1)[0][0]
                            wildcolor = most_common_color
                            print("Color changes to", wildcolor)
                            top_card.color = wildcolor
                            turn = 'Player'
                        
                else:
                    print('\nNAO wants to pull a card from deck')
                    time.sleep(1)
                    # NAO SPEECH
                    say = str("I want to pull a card from the deck")
                    nao_talk.run_given_speech(say, 5)
                    #temp_card = deck.deal() #here change with the computer vision
                    nao_move.move_RArm_withHead()
                    #detected_card = nao_vision.detect_card()
                    # nao_hand.add_card(detected_card) #actually we don't need to add this
                    # temp_card = detected_card
                    # NAO SPEECH
                    #say = str("I drew" + str(temp_card))
                    #nao_talk.run_given_speech(say,3)

                    if single_card_check(top_card, temp_card):
                        print('\nNAO has option to throw:{}'.format(temp_card))
                        time.sleep(1)
                        if temp_card.cardtype == 'number':
                            top_card = temp_card
                            # NAO Speech
                            say = str(str(top_card) + "is on the top.")
                            nao_talk.run_given_speech(say, 5)
                            turn = 'Player'
                        else:
                            if temp_card.rank == 'Skip':
                                # NAO SPEECH
                                event_type = 'skip_card'
                                nao_talk.run_speech(event_type)
                                nao_move.Left_Hand_Movement()
                                turn = 'NAO'
                                top_card = temp_card
                            elif temp_card.rank == 'Reverse':
                                # NAO SPEECH
                                event_type = 'reverse_card'
                                nao_talk.run_speech(event_type)
                                nao_move.Left_Hand_Movement()
                                turn = 'NAO'
                                top_card = temp_card
                            elif temp_card.rank == 'Draw2':
                                # player_hand.add_card(deck.deal())
                                # player_hand.add_card(deck.deal())
                                # NAO SPEECH
                                event_type = 'draw2_card'
                                nao_talk.run_speech(event_type)
                                nao_move.Left_Hand_Movement()
                                top_card = temp_card
                                turn = 'NAO'
                            elif temp_card.rank == 'Draw4':
                                # NAO SPEECH
                                event_type = 'draw4_card'
                                nao_talk.run_speech(event_type)
                                # for i in range(4):
                                #   player_hand.add_card(deck.deal())
                                nao_move.Left_Hand_Movement()
                                top_card = temp_card

                                draw4color = nao_hand.cards[0].color
                                print('Color changes to', draw4color)
                                # NAO Speech
                                say = str("Color changes to" + str(draw4color))
                                nao_talk.run_given_speech(say, 5)
                                top_card.color = draw4color
                                turn = 'NAO'

                            elif temp_card.rank == 'Wild':
                                # NAO SPEECH
                                event_type = 'wild_card'
                                nao_talk.run_speech(event_type)
                                nao_move.Left_Hand_Movement()
                                top_card = temp_card
                                wildcolor = nao_hand.cards[0].color
                                print("Color changes to", wildcolor)
                                # NAO SPEECH
                                say = str("Color changes to" + str(wildcolor))
                                nao_talk.run_given_speech(say, 5)
                                top_card.color = wildcolor
                                turn = 'Player'
                    else:
                        print('NAO doesnt have a card')
                        # NAO SPEECH
                        say = str("I don't have a card.")
                        nao_talk.run_given_speech(say,5)
                        time.sleep(1)
                        nao_move.move_RArm_withHead()
                        #detected_card = nao_vision.detect_card()
                        #temp_card = detected_card
                        nao_hand.add_card(temp_card)
                        turn = 'Player'
                #print('\nNAO has {} cards remaining'.format(nao_hand.no_of_cards()))
                # NAO SPEECH
                #say = str("I have "+ str(nao_hand.no_of_cards()) + "cards left.")
                #nao_talk.run_given_speech(say,5)
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


