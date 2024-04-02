#!/path to python interpreter

from asyncio import sleep
from google.cloud import speech
import io
import json
import pickle
from scipy.spatial.transform import Rotation as R
import simpleaudio as sa
import time

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool, String
from spetoge_msgs.msg import KeywordCheckerResponse

import subprocess

class DataDrivenNode():
    def __init__(self):
        self.keyword_checker_status = ""
        self.speech_generation_status = ""

        rospy.init_node('data_driven_node')
        rospy.loginfo("data_driven_node started")

        self.speech_to_gesture_path = rospy.get_param("speech_to_gesture_directory_path")

        # Subscriber
        self.sub_keyword_checker = rospy.Subscriber(
            "retail_robot/keyword_checker_response", KeywordCheckerResponse, callback=self.set_keyword_checker_status)
        self.sub_speech_generation = rospy.Subscriber(
            "retail_robot/speech_generation_status", String, callback=self.set_speech_generation_status)

        # Publisher
        self.finished_pub = rospy.Publisher(
            "retail_robot/execution_finished", Bool, queue_size=10)

        self.arm_left_pub = rospy.Publisher(
            "/arm_left_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
        self.arm_right_pub = rospy.Publisher(
            "/arm_right_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
        self.head_pub = rospy.Publisher(
            "/head_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)

        rospy.spin()

    def set_keyword_checker_status(self, status):
        if not status.deictic:
            self.keyword_checker_status = status.response
        else:
            return

        # start data driven approach if both status are set
        if self.keyword_checker_status == self.speech_generation_status:
            self.keyword_checker_status = False
            self.speech_generation_status = True
            self.execute_bvh_generation()

    def set_speech_generation_status(self, status):
        if status.data:
            self.speech_generation_status = status.data
        else:
            return

        # start data driven approach if both status are set
        if self.keyword_checker_status == self.speech_generation_status:
            self.keyword_checker_status = False
            self.speech_generation_status = True
            self.execute_bvh_generation()


    def execute_bvh_generation(self):
        rospy.loginfo("Start to generate a Trajectory with data driven approach!")

        # check if only rule based approach is used
        no_movement = False
        with open(self.speech_to_gesture_path + "/config/config.json") as json_file:
            config = json.load(json_file)
            robot_mode = config['robot_mode']
            if robot_mode == "only_rule_based":
                no_movement = True

        # Instantiates a client
        # we have to use the Google cloud speech api to get the word time offsets
        stt_client = speech.SpeechClient()

        audio_file_path = self.speech_to_gesture_path + "/config/output.wav"
        audio = speech.RecognitionAudio(uri=audio_file_path)

        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz = 24000,
            language_code = "en-US",
            enable_word_time_offsets = True
        )

        with io.open(audio_file_path, "rb") as f:
            content = f.read()
        audio = {"content": content}

        stt_response = stt_client.recognize(request={"config": config, "audio": audio})

        if not no_movement:
            self.generate_tsv_file(stt_response)
            # execute the neural network to get the bvh file of the human avatar
            self.generate_bvh()

            # the generation of the trajectory has to be done using python2
            p = subprocess.Popen(['/usr/bin/python', self.speech_to_gesture_path + '/src/execute_moveit.py',  self.speech_to_gesture_path])
            p.wait()

            # load trajectories and publish them
            with open(self.speech_to_gesture_path+ "/config/arm_left_trajectory.pkl", 'rb') as f:
                trajectory_arm_left = pickle.load(f)

            with open(self.speech_to_gesture_path+ "/config/arm_right_trajectory.pkl", 'rb') as f:
                trajectory_arm_right = pickle.load(f)

            self.arm_left_pub.publish(trajectory_arm_left)
            self.arm_right_pub.publish(trajectory_arm_right)
        else:
            # in this case no movement is executed (only rule based approach)
            time.sleep(30)  # to make time needed comparable to data driven approach

        # execute audio
        wave_obj = sa.WaveObject.from_wave_file(self.speech_to_gesture_path + "/config/output.wav")
        play_obj = wave_obj.play()
        play_obj.wait_done()

        # reset status
        self.keyword_checker_status = ""
        self.speech_generation_status = ""

        # send status when finished
        status = Bool()
        status.data = True
        self.finished_pub.publish(status)

    def generate_bvh(self):
        # Start Co-Speech Gesture Generation
        with open(self.speech_to_gesture_path + "/config/config.json", 'r') as f:
            config = json.load(f)
            venv_path = config["cospeech_gesture_generation_venv_path"]
            inference_path = config["cospeech_gesture_generation_inference_path"]
            network_path = config["cospeech_gesture_generation_network_path"]
            output_path = config["cospeech_gesture_generation_output_path"]

        p = subprocess.Popen([venv_path, inference_path, network_path, self.speech_to_gesture_path + "/config/output.tsv"])
        p.wait()

    def generate_tsv_file(self, stt_response):
        # generate tsv file from stt response
        # the tsv file is needed for the neural network
        tsv_string = ""

        for word in stt_response.results[0].alternatives[0].words:
            start_time = word.start_time.total_seconds()
            end_time = word.end_time.total_seconds()
            word_string = word.word
            new_line = '%.4f' % start_time + '\t' + '%.4f' % end_time + '\t' + word_string + '\n'
            
            tsv_string += new_line
        
        # save tsv file to disk
        with open(self.speech_to_gesture_path + "/config/output.tsv", 'w') as f:
            f.write(tsv_string)


if __name__ == '__main__':
    try:
        data_driven_node = DataDrivenNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Why did you kill me?")
