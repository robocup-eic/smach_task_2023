#
#* General
verbose = True
default_mic = True # on or off


#* ASR 
always_asr_url =  "http://localhost:5102/"
asr_url =  "http://localhost:5101/"
asr_model = "medium.en"

asr_mic_index= 4
asr_mic_name='USB Audio Device'

asr_energy_calibration =  True #
calibration = True
## These are the ones you should change
noise_reduction = True
energy_threshold =  450

calibration_coefficient =  1.5

# ChatGPT Keys
gpt3_5_turbo_key = "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"
gpt4_key = ""

#* Wakeword
wakeword_url =  "http://localhost:5100/"

wakeword_key = "A60C/RHA8NoFcfih1VnnW+MR736qtLekdo/2DrKSTx3dQnd4kmZBlw=="


mic_index_ww =  0
mic_name_ww = "Built-in Microphone"


#* TTS
tts_url =  "http://localhost:5003/tts"

azure_key = "11cf0c0fd3774358b647642975f821e7"


#* RASA NLU 
rasa_url =  "http://localhost:5005/webhooks/rest/webhook"
rasa_parse_url =  "http://localhost:5005/model/parse"
rasa_actions_url =  "http://localhost:5055"

gpt_key = "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"

""" 
ASR Notes 
- Calibration: Auto calibrate first then grab the number 
- list of Whisper Model 
    "tiny.en"
    "tiny"
    "base.en"
    "base"
    "small.en"
    "small"
    "medium.en"
    "medium"
    "large-v1"
    "large-v2"
"""



