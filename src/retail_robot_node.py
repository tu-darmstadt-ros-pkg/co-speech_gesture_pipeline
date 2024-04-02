#!/path to python interpreter

import rospy
from robotrust_msgs.srv import ExecuteGesture
from spetoge_msgs.msg import GPTRequest
from std_msgs.msg import String, Bool
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep

from google.cloud import speech
import queue

import pyaudio

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms


class RetailRobotNode():
    def __init__(self):
        rospy.init_node('retail_robot_node')
        rospy.loginfo("retail_robot_node started")

        self.sub = rospy.Subscriber(
            "retail_robot/execution_finished", Bool, callback=self.finished)
        self.pub_gpt_request = rospy.Publisher(
            "retail_robot/gpt_request", GPTRequest, queue_size=10)

        # move torso up
        self.torso_pub = rospy.Publisher(
            "/torso_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
        sleep(1)
        
        torso_follow_joint_traj_action_goal = FollowJointTrajectoryActionGoal()
        torso_follow_joint_traj_action_goal.goal.trajectory.joint_names.append("torso_lift_joint")

        joint_traj_point = JointTrajectoryPoint()
        joint_traj_point.positions = [0.4]
        joint_traj_point.time_from_start = rospy.Time.from_sec(1)

        torso_follow_joint_traj_action_goal.goal.trajectory.points.append(joint_traj_point)

        self.torso_pub.publish(torso_follow_joint_traj_action_goal)


        # init service
        self.execute_gesture_service_exists = False
        rospy.wait_for_service("/gesture_execution/execute_tiago_gesture")
        self.execute_tiago_gesture_service = rospy.ServiceProxy(
            "/gesture_execution/execute_tiago_gesture", ExecuteGesture)
        self.execute_gesture_service_exists = True

        self.listen_to_mic()
        rospy.spin()

    def finished(self, status):
        rospy.loginfo("Execution finished")
        
        # get back to neutral position
        if self.execute_gesture_service_exists:
            self.execute_tiago_gesture_service(["neutral"], 1, True)
        else:
            rospy.loginfo("unable to executing trajectory, because service does not exist")

        # listen to mic again for new question
        self.listen_to_mic()

    def listen_to_mic(self):
        for _ in range(100):
            print(" ----------------- ")
        print("Ask NOW a question")
        language_code = "en-US"  # a BCP-47 language tag

        client = speech.SpeechClient()
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code,
        )

        streaming_config = speech.StreamingRecognitionConfig(
            config=config, interim_results=False
        )

        with MicrophoneStream(RATE, CHUNK) as stream:
            audio_generator = stream.generator()
            requests = (
                speech.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )

            responses = client.streaming_recognize(streaming_config, requests)
            transcript = next(responses).results[0].alternatives[0].transcript
            print(transcript)
            
            self.pub_gpt_request.publish(transcript)

class MicrophoneStream:
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(
            self: object,
            rate: int = RATE,
            chunk: int = CHUNK
    ) -> None:
        """The audio -- and generator -- is guaranteed to be on the main thread.
        """
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self: object) -> object:
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(
            self: object,
            type: object,
            value: object,
            traceback: object,
    ) -> None:
        """Closes the stream, regardless of whether the connection was lost or not."""
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(
            self: object,
            in_data: object,
            frame_count: int,
            time_info: object,
            status_flags: object,
    ) -> object:
        """Continuously collect data from the audio stream, into the buffer.

        Args:
            in_data: The audio data as a bytes object
            frame_count: The number of frames captured
            time_info: The time information
            status_flags: The status flags

        Returns:
            The audio data as a bytes object
        """
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self: object) -> object:
        """Generates audio chunks from the stream of audio data in chunks.

        Args:
            self: The MicrophoneStream object

        Returns:
            A generator that outputs audio chunks.
        """
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)


if __name__ == '__main__':
    try:
        retail_robot_node = RetailRobotNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Why did you kill me?")
