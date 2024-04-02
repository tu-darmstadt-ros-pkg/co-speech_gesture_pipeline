#!/path to python interpreter

from google.cloud import texttospeech

import rospy
from std_msgs.msg import String
from spetoge_msgs.msg import GPTResponse

class SpeechGenerationNode():
    def __init__(self):
        rospy.init_node('speech_generation_node')
        rospy.loginfo("speech_generation_node started")

        self.directory_path = rospy.get_param("speech_to_gesture_directory_path")

        self.sub = rospy.Subscriber("retail_robot/gpt_response", GPTResponse, callback=self.generate_speech)
        self.pub = rospy.Publisher("retail_robot/speech_generation_status", String, queue_size=10)

        rospy.spin()

    def generate_speech(self, gpt_response: GPTResponse):
        text = gpt_response.response

        # # google text-to-speech API integration
        # for authentication load the Google credentials .json file in the bashrc

        # Instantiate a client
        tts_client = texttospeech.TextToSpeechClient()


        # Set the text input to be synthesized
        synthesis_input = texttospeech.SynthesisInput(text=text)

        # Build the voice request, select the language code ("en-US") and the ssml
        voice = texttospeech.VoiceSelectionParams(
            language_code="en-US", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL
        )

        # Select the type of audio file you want returned
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            speaking_rate=0.75
        )

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        tts_response = tts_client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        # The response's audio_content is binary
        with open(self.directory_path + "/config/output.wav", "wb") as out:
            # Write the response to the output file.
            out.write(tts_response.audio_content)
            rospy.loginfo('Audio content written to file "output.wav"')


        # set status to true if mp3 is writen
        status = String()
        status.data = text
        self.pub.publish(status)
        

if __name__ == '__main__':
    try:
        SpeechGenerationNode()
    except:
        rospy.loginfo("Error in speech generation")