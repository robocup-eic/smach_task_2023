# run with conda env: base
import nlp_client
import threading
import signal
import time
import os
import smach
import os
import signal
import smach
from ratfin import *
import simpleaudio as sa  
# from utils import Speak

# Task specific state
class StopEnd(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1'],
                            input_keys=['stop'],
                            output_keys=['stop'])
        pass


    def execute(self, userdata):
        userdata.stop = True
        return 'out1'

class KillProcessState(smach.State):
    """ smach.StateMachine.add('KILLPROCESS',
                            KillProcessState(),
                            transitions={'out1': 'out0'}
                                        ) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        pid = os.getpid()
        try:
            os.kill(pid, signal.SIGKILL)
            return 'success'
        except ProcessLookupError:
            print(f"Process with id {pid} does not exist.")
            return 'failure'
        except PermissionError:
            print(f"You don't have permission to kill process with id {pid}.")
            return 'failure'

    
class EmergencyStop():
    """ How to use
    1. Create an instance of EmergencyStop
    2. Create a thread to run the execute function
    3. Call the emer_stop_handler function in the main thread
    
    >>> es = EmergencyStop()
    >>> es_thread = threading.Thread(target=es.execute)
    >>> es_thread.start()
    >>> es.emer_stop_handler() 
    """
    def __init__(self):
        self.stop_flag = False


    def execute(self):
        print('(EmergencyStop): Listening for Walkie Freeze')
        if nlp_client.ww_listen(text="walkie_freeze", log=True):
            wave_obj = sa.WaveObject.from_wave_file("/home/walkie/smach_task_2023/src/core_nlp/GTA_Failed.wav")
            play_obj = wave_obj.play()
            # nlp_client.speak('fuck fuck fuck fuck fuck stopping!')
            nlp_client.speak('god damn it fuck.')
            #play GTA_Failed.mp3 using sa
            self.stop_flag = True
        
        print('EmergencyStop detected')

        return "out1"
    def emer_stop_handler(self):
        while True:
            pid = os.getpid()
            if self.stop_flag:
                # Speak(text='fuck fuck fuck fuck fuck fuck fuck fuck').execute(userdata='')
                print("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk ;)")
                pid = pid  # Replace with your process id
                try:
                    os.kill(pid, signal.SIGKILL)
                    
                except ProcessLookupError:
                    print(f"Process with id {pid} does not exist.")
                except PermissionError:
                    print(f"You don't have permission to kill process with id {pid}.")
                break
            time.sleep(0.1)


# function to run emergencystop
def listen_for_kill_command():
    """ 
    Listens for "Walkie Freeze" this will kill the program """
    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()
    return True

def main():

    x = EmergencyStop()

    x.execute()

    
if __name__ == '__main__':
    main()
