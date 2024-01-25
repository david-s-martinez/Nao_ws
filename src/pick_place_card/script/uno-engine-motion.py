#!/usr/bin/env python
import random
import time
from collections import Counter
from move_to_object import NAOMove
from naoqi_bridge_msgs.msg import SpeechWithFeedbackActionGoal,WordRecognized
import rospy
from nao_communication import NAOCommunicate, NAOVision

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
            # print(hand.cardsstr.index(str(c))+1))
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

def see_card(nao_see, nao_talk):
    while 1:
        nao_talk.run_given_speech("I will see my card",delay=3)
        card = nao_see.get_top_card()

        if card:

            nao_talk.run_given_speech("Is my card correct: {}".format(card), delay=4)
            # response = raw_input("\nIs my card correct: {} y/n".format(card))
            # bumper = ""
            # if nao_talk.bump.state == 1:
            #     if nao_talk.bump.bumper == 1:
            #         bumper = "left"
            #     else:
            #         bumper = "right"
            while nao_talk.head.state == 0:
                pass
            if nao_talk.head.button == 1 and nao_talk.head.state == 1:
                print("Thank you, my card is {}".format(card))
                nao_talk.run_given_speech("Thank you, my card is {}".format(card), delay=3)
                break
            elif nao_talk.head.button == 3 and nao_talk.head.state == 1:
                nao_talk.run_given_speech("Show me my card again",delay=3)
                print("Show me my card again")
                rospy.sleep(2)
            # if response == "y" or nao_talk.head.state == 1:
            #     break
            # elif response == "n" or nao_talk.head.state == 3:
            #     print("Show me the card again")
            #     rospy.sleep(2)

    print('card type', card,type(card))
    if card[0] == 'Colorless':
        card[0] == None
    
    card = Card(card[0],card[1])
    # nao_hand.add_card(card) # i.e. append

    return card

def see_top_card(nao_see, nao_talk):
    while 1:
        nao_talk.run_given_speech("I'll take a look at the top card",delay=4)
        card = nao_see.get_top_card()

        if card is not []:

            nao_talk.run_given_speech("Is the top card correct: {}".format(card), delay=4)
            # response = raw_input("\nIs my card correct: {} y/n".format(card))
            # bumper = ""
            # if nao_talk.bump.state == 1:
            #     if nao_talk.bump.bumper == 1:
            #         bumper = "left"
            #     else:
            #         bumper = "right"
            while nao_talk.head.state == 0:
                pass
            if nao_talk.head.button == 1 and nao_talk.head.state == 1:
                print("Thank you, the top card is {}".format(card))
                nao_talk.run_given_speech("Thank you, the top card is {}".format(card), delay=3)
                break
            elif nao_talk.head.button == 3 and nao_talk.head.state == 1 or card == []:
                nao_talk.run_given_speech("Show me the top card again",delay=3)
                print("Show me the card again")
                rospy.sleep(2)
            # if response == "y" or nao_talk.head.state == 1:
            #     break
            # elif response == "n" or nao_talk.head.state == 3:
            #     print("Show me the card again")
            #     rospy.sleep(2)

    print(card)
    if card[0] == 'Colorless':
        card[0] == None
    card = Card(card[0],card[1])
    # nao_hand.add_card(card) # i.e. append

    return card

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
    nao_see = NAOVision()
    player_hand = NAOHand()
    #deck = Deck()
    nao_move = NAOMove(needs_node=False)

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
        for counter_cards in range(3):

        # Computer Vision Algorithm 
            # while nao_talk.bump.state == 0 :
            #     pass
            # if nao_talk.bump.bumper == 0:
            #     rospy.sleep(2)
            #     print('Right foot pressed')
            card = see_card(nao_see, nao_talk)
            nao_hand.add_card(card)
                ####
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



        top_card = see_top_card(nao_see, nao_talk)
        if top_card.cardtype != 'number':
            while top_card.cardtype != 'number':
                print("Top card is not valid")
                top_card = see_top_card(nao_see, nao_talk)
            

        print('\nStarting Card is: {}'.format(top_card))
        time.sleep(1)
        # NAO SPEECH 
        say = str("Starting Card is" + str(top_card))
        nao_talk.run_given_speech(say,3)
        playing = True

        player_no_cards = 3

        


        while playing:

            if turn == 'Player':
                #NAO SPEECH 
                
                nao_talk.run_given_speech("It's your turn",delay=2)

                throw_pulled = ""
                # NAO LISTEN
                # choice = raw_input("\nHit or Pull? (h/p): ")  # Replace by Speech recognition module
                nao_talk.run_given_speech("Wanna hit or pull?",delay=2)
                while nao_talk.head.state == 0:
                    pass
                
                if nao_talk.head.button == 3:
                    player_no_cards += 1
                    # NAO LISTEN
                    nao_talk.run_given_speech("you're pulling a card", delay=2)

                    while nao_talk.head.state == 0:
                        pass

                    nao_talk.run_given_speech("Wanna throw this card?", delay=2)

                    if nao_talk.head.button == 1:
                        throw_pulled = "yes"
                    
                    turn = 'NAO'
                
                if nao_talk.head.button == 1 or throw_pulled == "yes":
                    player_no_cards -= 1
                    nao_talk.run_given_speech("So you're hitting a card.",delay=2)
                    
                    top_card = see_top_card(nao_see, nao_talk)
                    if top_card.cardtype == 'number':
                        turn = 'NAO'
                    else:
                        if top_card.rank == 'Skip':
                            turn = 'Player'

                        elif top_card.rank == 'Reverse':
                            turn = 'Player'

                        elif top_card.rank == 'Draw2':
                            for i in range(2):
                                nao_move.move_RArm_withHead()
                                nao_hand.add_card(see_card(nao_see, nao_talk))
                            turn = 'Player'

                        elif top_card.rank == 'Draw4':
                            for i in range(4):
                                nao_move.move_RArm_withHead()
                                nao_hand.add_card(see_card(nao_see, nao_talk))
                    
                            nao_talk.run_given_speech("Please change color in the prompt",delay=4)
                            new_color = raw_input('Change color to (enter in caps): ')
                            if new_color != new_color.upper():
                                new_color = new_color.upper()
                            top_card.color = new_color
                            turn = 'NAO'

                        elif top_card.rank == 'Wild':
                            nao_talk.run_given_speech("Please change color in the prompt",delay=4)
                            new_color = raw_input('Change color to (enter in caps): ')
                            if new_color != new_color.upper():
                                new_color = new_color.upper()
                            top_card.color = new_color
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
                        # NAO Speech
                        say = str("My last card is Special card, Let me draw.")
                        nao_talk.run_given_speech(say, 5)
                        nao_move.move_RArm_withHead()
                        nao_hand.add_card(see_card(nao_see, nao_talk))
                    else:
                        say = str("UNO")
                        nao_talk.run_given_speech(say, 5)

                temp_card = full_hand_check(nao_hand, top_card)
                print(nao_hand)
                print(top_card)
                print(temp_card)
                if temp_card != 'no card':
                    # here nao will ask user to put desired card on shelf, then nao will pick from shelf to board.
                    nao_talk.run_given_speech("Please prepare {}".format(temp_card),5)
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
                            # NAO SPEECH
                            event_type = 'draw2_card'
                            nao_talk.run_speech(event_type)
                            top_card = temp_card
                            turn = 'NAO'
                        elif temp_card.rank == 'Draw4':
                            # NAO SPEECH
                            event_type = 'draw4_card'
                            top_card = temp_card
                            nao_talk.run_speech(event_type)
                            # Assuming nao_hand.cards is a list of card objects with a 'color' attribute
                            colors = []
                            for card in nao_hand.cards:
                                if card.color is None:
                                    continue 
                                else:
                                    colors.append(card.color)

                            print(colors)

                            # Use Counter to count occurrences of each color
                            color_counts = Counter(colors)
                            # Find the color with the maximum occurrence
                            most_common_color = color_counts.most_common(1)[0][0]
                            if most_common_color is None :
                                new_color = color_counts.most_common(2)[0][0]
                            else:
                                new_color = color_counts.most_common(1)[0][0]
                            #new_color = nao_hand.cards[0].color
                            print('Color changes to', new_color)
                            say = str("Color changes to" + str(new_color))
                            nao_talk.run_given_speech(say, 5)
                            top_card.color = new_color
                            turn = 'Player'
                          
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
                            if most_common_color is None :
                                new_color = color_counts.most_common(2)[0][0]
                            else:
                                new_color = most_common_color
                            print("Color changes to", new_color)
                            say = str("Color changes to" + str(new_color))
                            nao_talk.run_given_speech(say, 5)
                            top_card.color = new_color
                            turn = 'Player'
                        
                else:
                    print('\nNAO wants to pull a card from deck')
                    time.sleep(1)
                    # NAO SPEECH
                    say = str("No valid card, let me draw")
                    nao_talk.run_given_speech(say, 5)
                    #temp_card = deck.deal() #here change with the computer vision
                    nao_move.move_RArm_withHead()
                    #detected_card = nao_vision.detect_card()
                    temp_card = see_card(nao_see, nao_talk) #actually we don't need to add this
                    # temp_card = detected_card
                    # NAO SPEECH
                    say = str("I drew" + str(temp_card))
                    nao_talk.run_given_speech(say,3)
                    print(say)

                    if single_card_check(top_card, temp_card):
                        print('\nNAO has option to throw:{}'.format(temp_card))
                        nao_talk.run_given_speech("Please prepare {}".format(temp_card),5)
                        nao_move.Left_Hand_Movement()
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
                                #nao_move.Left_Hand_Movement()
                                turn = 'NAO'
                                top_card = temp_card
                            elif temp_card.rank == 'Reverse':
                                # NAO SPEECH
                                event_type = 'reverse_card'
                                nao_talk.run_speech(event_type)
                                #nao_move.Left_Hand_Movement()
                                turn = 'NAO'
                                top_card = temp_card
                            elif temp_card.rank == 'Draw2':
                                # NAO SPEECH
                                event_type = 'draw2_card'
                                nao_talk.run_speech(event_type)
                                #nao_move.Left_Hand_Movement()
                                top_card = temp_card
                                turn = 'Player'
                            elif temp_card.rank == 'Draw4':
                                # NAO SPEECH
                                event_type = 'draw4_card'
                                nao_talk.run_speech(event_type)
                                # for i in range(4):
                                #   player_hand.add_card(deck.deal())
                                #nao_move.Left_Hand_Movement()
                                top_card = temp_card

                                most_common_color = color_counts.most_common(1)[0][0]
                                if most_common_color is None :
                                    new_color = color_counts.most_common(2)[0][0]
                                else:
                                    new_color = most_common_color
                                print('Color changes to', new_color)
                                # NAO Speech
                                say = str("Color changes to" + str(new_color))
                                nao_talk.run_given_speech(say, 5)
                                top_card.color = new_color
                                turn = 'Player'

                            elif temp_card.rank == 'Wild':
                                # NAO SPEECH
                                event_type = 'wild_card'
                                nao_talk.run_speech(event_type)
                                #nao_move.Left_Hand_Movement()
                                top_card = temp_card
                                most_common_color = color_counts.most_common(1)[0][0]
                                if most_common_color is None :
                                    new_color = color_counts.most_common(2)[0][0]
                                else:
                                    new_color = most_common_color
                                print("Color changes to", new_color)
                                # NAO SPEECH
                                say = str("Color changes to" + str(new_color))
                                nao_talk.run_given_speech(say, 5)
                                top_card.color = new_color
                                turn = 'Player'
                    else:
                    #     print('NAO doesnt have a card')
                    #     # NAO SPEECH
                    #     say = str("I don't have a card, I will draw")
                    #     nao_talk.run_given_speech(say,5)
                    #     time.sleep(1)
                    #     nao_move.move_RArm_withHead()
                    #     #detected_card = nao_vision.detect_card()
                    #     #temp_card = detected_card
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

        say = str("Would you like to play again?")
        nao_talk.run_given_speech(say,5)
        new_game = raw_input('Would you like to play again? (y/n)')
        if new_game == 'y':
            continue
        else:
            print('\nThanks for playing!!')
            say = str("Thanks for playing!!")
            nao_talk.run_given_speech(say,5)
            break    


if __name__ == '__main__':
	main()


