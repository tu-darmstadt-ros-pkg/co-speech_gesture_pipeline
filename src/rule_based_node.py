#!/path to python interpreter

import math
import simpleaudio as sa
from time import sleep

import rospy
from robotrust_msgs.srv import ExecuteGesture
from std_msgs.msg import Bool, String
from spetoge_msgs.msg import KeywordCheckerResponse


class RuleBasedNode():
    def __init__(self):
        self.keyword_checker_response = None
        self.keyword_checker_status = ""
        self.speech_generation_status = ""

        rospy.init_node('rule_based_node')
        rospy.loginfo("rule_based_node started")

        self.robotrust_gesture_execution_path = rospy.get_param("robotrust_gesture_execution_path")
        self.speech_to_gesture_path = rospy.get_param("speech_to_gesture_directory_path")

        # Subscriber
        self.sub_keyword_checker = rospy.Subscriber(
            "retail_robot/keyword_checker_response", KeywordCheckerResponse, callback=self.set_keyword_checker_status)
        self.sub_speech_generation = rospy.Subscriber(
            "retail_robot/speech_generation_status", String, callback=self.set_speech_generation_status)

        # Publisher
        self.finished_pub = rospy.Publisher(
            "retail_robot/execution_finished", Bool, queue_size=10)
        
        # init service for executing gesture
        self.execute_gesture_service_exists = False
        rospy.wait_for_service("/gesture_execution/execute_tiago_gesture")
        self.execute_tiago_gesture_service = rospy.ServiceProxy(
            "/gesture_execution/execute_tiago_gesture", ExecuteGesture)
        self.execute_gesture_service_exists = True

        rospy.spin()

    def set_keyword_checker_status(self, status: KeywordCheckerResponse):
        if status.deictic:
            self.keyword_checker_status = status.response
            self.keyword_checker_response = status
        else:
            return

        if self.keyword_checker_status == self.speech_generation_status:
            self.execute()

    def set_speech_generation_status(self, status: String):
        if status.data:
            self.speech_generation_status = status.data
        else:
            return

        if self.keyword_checker_status == self.speech_generation_status:
            self.execute()

    def execute(self):
        # generate and write file for executing the gesture
        response = self.keyword_checker_response

        pos_x = response.pos_x
        pos_y = response.pos_y
        pos_z = response.pos_z

        alpha = -10 * math.sqrt(pos_x**2 + pos_y**2) + 80
        alpha = max(20, alpha)
        alpha = min(70, alpha)

        if pos_x <= 0: # use right arm to point at object #
            theta = -math.degrees(math.atan(pos_y / pos_x))
            self.write_dat_file(theta=theta, alpha=alpha, use_right_arm=True)
        elif pos_x > 0: # use left arm
            theta = math.degrees(math.atan(-pos_y / pos_x))
            self.write_dat_file(theta=theta, alpha=alpha, use_right_arm=False)

        # wait for 30 seconds to make it comparable to data driven approach
        sleep(30)

        # execute gesture and audio
        # gesture
        if self.execute_gesture_service_exists:
            self.execute_tiago_gesture_service(["rule_based"], 1, True)
        else:
            rospy.loginfo("unable to executing trajectory, because service does not exist")

        # play back audio file
        sleep(1.5)
        wave_obj = sa.WaveObject.from_wave_file(self.speech_to_gesture_path + "/config/output.wav")
        play_obj = wave_obj.play()
        play_obj.wait_done()
        sleep(1.5)

        self.keyword_checker_status = ""
        self.speech_generation_status = ""

        # send status when finished
        status = Bool()
        status.data = True
        self.finished_pub.publish(status)


    def write_dat_file(self, theta, alpha, use_right_arm):
        # initialize with neutral pose
        left_joint_angles = [-35.0, 85.0, 140.0, 100.0, -115.0, 30.0, -40.0]
        right_joint_angles = [-35.0, 85.0, 140.0, 100.0, -115.0, 30.0, -40.0]

        if use_right_arm:
            right_joint_angles = [theta, alpha, 0, alpha, 0, 0, 0]
        else: # use left arm
            left_joint_angles = [theta, alpha, 0, alpha, 0, 0, 0]

        # write the joint angles to dat file
        trajectory_path = self.robotrust_gesture_execution_path + '/data/tiago/rule_based/time_1.0.dat'
        with open(trajectory_path, 'w') as f:
            f.write("DONT_CORRECT_VALUES\n\n")
            for idx, angle in enumerate(right_joint_angles):
                f.write("right_joint{0}\n".format(idx+1))
                f.write("{0}\n\n".format(angle))
            f.write("\n")
            for idx, angle in enumerate(left_joint_angles):
                f.write("left_joint{0}\n".format(idx+1))
                f.write("{0}\n\n".format(angle))


if __name__ == '__main__':
    try:
        rule_based_node = RuleBasedNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Why did you kill me?")
