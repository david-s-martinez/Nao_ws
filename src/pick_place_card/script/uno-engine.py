import random
import time
from naoqi_bridge_msgs.msg import SpeechWithFeedbackActionGoal,WordRecognized
import rospy

class NAOTalk:

    def __init__(self):

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
            
            talk_line = random.choice(self.talk_phrases[event_type])
            
            # Construct the speech goal message
            talk_msg = SpeechWithFeedbackActionGoal()
            talk_msg.goal_id.id = str(rospy.Time.now().to_sec())
            talk_msg.goal.say = talk_line
            
            # Publish the trash talk line
            self.speech_pub.publish(talk_msg)       

    



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


class Deck:

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
        return self.deck.pop()

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
        return 'Player'
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


nao_hand = NAOHand()
top_card = None
nao_talk = NAOTalk()
player_hand = NAOHand()
deck = Deck()

while True:

    # NAO SPEECH 
    event_type = 'starter'
    nao_talk.run_speech(event_type)

    ###### Computer vision system integration here
    ### Let's say there is a function for detecting the cards
    #top_card = get_top_card()
    #new_cards = get_new_cards()
    ############
    #for card in new_cards:
        #nao_hand.add_card(card)
    

    deck.shuffle()

    for i in range(7):
        player_hand.add_card(deck.deal())
    for i in range(7):
        nao_hand.add_card(deck.deal())

    top_card = deck.deal()
    print('\nStarting Card is: {}'.format(top_card))
    time.sleep(1)
    # NAO SPEECH 
    event_type = 'draw_cards'
    nao_talk(event_type)
    playing = True

    turn = choose_first()
    print(turn + ' will go first')


    while playing:




        # make sure starting card is not an action card.
        if top_card.cardtype != 'number':
            while top_card.cardtype != 'number':
                # This should be top card recognition on the deck
                #top_card = get_new_top_card()
                top_card = deck.deal()
                pass

        # NAO's turn logic
        if nao_hand.no_of_cards() == 1:
            if last_card_check(nao_hand):
                print('Last card cannot be an action card \nAdding one card to NAO\'s hand')
                # NAO SPEECH 
                event_type = 'last_card_action'
                nao_talk(event_type)
                # Add a card to NAO's hand based on vision
                nao_hand.add_card(get_new_card())
                pass

        temp_card = full_hand_check(nao_hand, top_card)
        if temp_card != 'no card':

            print(' \nNAO throws:'.format(temp_card))
            nao_hand.remove_card(temp_card)
            top_card = temp_card  # Update the top card to the one NAO played
            # NAO SPEECH 
            event_type = 'throw_card'
            nao_talk(event_type)



        else:
            
            print('\nNAO needs to draw a card')
            # NAO SPEECH 
            event_type = 'draw_cards'
            nao_talk(event_type)
            # NAO draws a card based on computer vision input
            new_card = get_new_card()
            nao_hand.add_card(new_card)
            pass

        # Check if NAO wins
        if win_check(nao_hand):
            print('\nNAO WON!!')
            # NAO SPEECH 
            event_type = 'winning'
            nao_talk(event_type)
            break
