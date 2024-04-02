#!/path to python interpreter
import json
import time
import openai
from pathlib import Path
import os
import rospy
from std_msgs.msg import String
from spetoge_msgs.msg import GPTRequest, GPTResponse


class GPTNode():
    def __init__(self):
        rospy.init_node('gpt_node')
        rospy.loginfo("gpt_node started")

        self.directory_path = rospy.get_param("speech_to_gesture_directory_path")

        self.context = None
        self.enable_openai_key()
        self.load_context()

        self.sub = rospy.Subscriber(
            "retail_robot/gpt_request", GPTRequest, callback=self.generate_response)
        self.pub = rospy.Publisher(
            "retail_robot/gpt_response", GPTResponse, queue_size=10)

        # create transcript file and write it to config/transcript_files
        # generate new transcript file by indexing the name 
        # of the last transcript file and incrementing the index
        index = 0
        while True:
            path = self.directory_path + '/config/transcript_files/transcript_' + str(index) + '.txt'
            if not os.path.exists(path):
                break
            index += 1
        self.transcript_path = path
        with open(self.transcript_path, 'w') as f:
            f.write('Transcript file ' + str(index) + '\n')
            f.write("We start with a new experiment.\n")
            # write current date and time
            f.write(str(time.strftime("%Y-%m-%d %H:%M:%S")))
            f.write('\n\n')
            f.write('#' * 80)
            f.write('\n\n')
        rospy.spin()

    def generate_response(self, gpt_request: GPTRequest):
        rospy.loginfo("GPT REQUEST: %s", gpt_request.request)

        gpt_response = GPTResponse()

        prompt = "Answer the question as truthfully as possible."
        prompt += '\n\nContext: \n' + self.context + '\n\n'
        prompt += 'Q: ' + gpt_request.request + '\n'
        prompt += 'A:'

        gpt_result = openai.Completion.create(
            prompt=prompt,
            temperature=0,
            max_tokens=100,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0,
            model='text-davinci-003')
        gpt_response.response = gpt_result["choices"][0]["text"]

        rospy.loginfo("GPT RESPONSE: %s", gpt_response.response)
        self.pub.publish(gpt_response)

        # get attribut robot_mode from config file
        with open(self.directory_path + '/config/config.json', 'r') as f:
            config = json.load(f)
            robot_mode = config['robot_mode']

        # store response in file
        with open(self.transcript_path, 'a') as response_file:
            # write current date and time
            response_file.write('\n\n')
            response_file.write(str(time.strftime("%Y-%m-%d %H:%M:%S")))
            response_file.write('\n\n')
            response_file.write('Robot mode: ' + robot_mode + '\n')
            response_file.write('\n\n')
            # write request
            response_file.write('Q: ' + gpt_request.request + '\n')
            # write response
            response_file.write('A: ' + gpt_response.response + '\n')
            response_file.write(
                '----------------------------------------------------------------------------------------'
            )


    def enable_openai_key(self):
        openai_key_path = self.directory_path + '/config/openai_key.txt'
        with open(openai_key_path, 'r') as key_file:
            s = key_file.read()
            s = s[:-1]
            openai.api_key = s

    def load_context(self):
        # load context from file
        context_path = self.directory_path + '/config/retail_store_context_3.txt'
        with open(context_path, 'r') as context_file:
            self.context = context_file.read()


if __name__ == '__main__':
    try:
        gpt_node = GPTNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Why did you kill me?")
