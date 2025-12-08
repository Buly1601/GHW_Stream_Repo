from iot_control import IOTConnection
import cv2
import mediapipe as mp


class HandTracker(IOTConnection):

    def __init__(self, mode=False, max_hands=2, detection_con=0.5, model_complexity=1, track_con=0.5):
        #*2 call parent's init
        super().__init__()
        # variables
        self.mode = mode
        self.max_hands = max_hands
        self.detection_con = detection_con
        self.model_complexity = model_complexity
        self.track_con = track_con
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(self.mode, self.max_hands, self.model_complexity, self.detection_con, self.track_con)
        self.mp_draw = mp.solutions.drawing_utils
        self.command = None # useful one
        self.prev_command = None


    def _hand_finder(self, image, draw=True):
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)

        if self.results.multi_hand_landmarks:
            for lm in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(image, lm, self.mp_hands.HAND_CONNECTIONS)
        
        return image


    def _position_finder(self, image, hand_no=0, draw=True):
        self.lm_list = []
        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_no]
            for id, lm in enumerate(hand.landmark):
                h, w, _ = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lm_list.append([id, cx, cy])
                
                if id == 0 and draw:
                    cv2.circle(image, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
                

    def _finger_down(self, fingers=[]):
        """
        Returns True if fingers are down relative to the palm [0]
        """
        heights = []
        for finger in fingers:
            if finger == 4:
                heights.append(True if abs(self.lm_list[finger][2] - self.lm_list[0][2]) < 100 else False)
            else:
                heights.append(True if abs(self.lm_list[finger][2] - self.lm_list[0][2]) < 90 else False)

        return heights
    

    def _gesture_command(self):
        """
        Gets the command from the position of tips relative to 0.
        Relate to README file for command ilustration.
        """
        if self.lm_list:
            # Turn Bedroom light on (index finger only)
            if abs(self.lm_list[8][2] - self.lm_list[0][2]) >= 150:
                h = self._finger_down(fingers=[20, 16, 12, 4])
                if len(set(h)) == 1 and h[0] == True:
                    #print("Bedroom Light ON")
                    self.command = "BL1"

            # Turn Bedroom light off (all fingers down)
            if len(set(self._finger_down(fingers=[4, 8, 12, 16, 20]))) == 1 and self._finger_down(fingers=[4, 12, 16, 20])[0] == True:
                #print("Bedroom Light OFF")
                self.command = "BL0"

            # turn garage light on ("2" with index and middle)
            if abs(self.lm_list[12][2] - self.lm_list[0][2]) >= 200:
                h = self._finger_down(fingers=[20, 16, 4])
                if len(set(h)) == 1 and h[0] == True:
                    #print("Garage Light ON")
                    self.command = "GL1"

            # turn garage light off (only pinky)
            if abs(self.lm_list[20][2] - self.lm_list[0][2]) >= 135:
                h = self._finger_down(fingers=[8, 16, 12, 4])
                if len(set(h)) == 1 and h[0] == True:  
                    #print("Garage Light OFF")
                    self.command = "GL0"

            # clockwise servo (open garage door) (spiderman hand)
            if abs(self.lm_list[8][2] - self.lm_list[0][2]) >= 150 and abs(self.lm_list[20][2] - self.lm_list[0][2]) >= 135:
                h = self._finger_down(fingers=[12, 16])
                if len(set(h)) == 1 and h[0] == True:
                    #print("Garage Door Open")
                    self.command = "GD1"
            
            # anticlockwise servo (garage door close) (only thumb)
            if abs(self.lm_list[4][2] - self.lm_list[0][2]) >= 100:
                h = self._finger_down(fingers=[12, 16])
                if len(set(h)) == 1 and h[0] == True:
                    #print("Closing Garage Door")
                    self.command = "GD0"
            
            # check if command is not repeated
            if self.command != self.prev_command:
                self.prev_command = self.command
                # send command here
                print(self.command)
                #*2 send message to broker 
                self._send_to_topic(self.command) 
            
    
    def main(self, show=True):
        """
        Main command that runs the class and tracks hands with 
        helper methods.
        """
        cap = cv2.VideoCapture(0)

        while True:
            success, frame = cap.read()
            if not success:
                break
            # detect & draw landmarks
            frame = self._hand_finder(frame)
            # update landmark list
            self._position_finder(frame)
            # print command if detected     
            self._gesture_command()          

            # show if true
            if show:
                cv2.imshow("Hand Tracking", frame)
            # close when pressed q
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # destroy windows
        cap.release()
        cv2.destroyAllWindows()
        

if __name__ == "__main__":
    tracker = HandTracker()
    tracker.main()