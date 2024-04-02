#!/path to python interpreter

import json
import inflect

import rospy
from spetoge_msgs.msg import GPTResponse, KeywordCheckerResponse

def count_keywords(string, keywords):
    c = 0
    for keyword in keywords:
        if keyword.lower() in string.lower():
            c += 1
    return c


class KeywordCheckerNode():
    def __init__(self):
        rospy.init_node('keyword_checker_node')
        rospy.loginfo("keyword_cheker_node started")

        self.directory_path = rospy.get_param("speech_to_gesture_directory_path")

        self.keyword_objects = None
        self.load_keywords_objects()

        self.sub = rospy.Subscriber("retail_robot/gpt_response", GPTResponse, callback=self.check_gpt_response)
        self.pub = rospy.Publisher("retail_robot/keyword_checker_response", KeywordCheckerResponse, queue_size=10)

        rospy.spin()

    def check_gpt_response(self, gpt_response: GPTResponse):
        rospy.loginfo("KEYWORDCHECKER REQUEST: %s", gpt_response.response)

        always_data_driven = False
        with open(self.directory_path + '/config/config.json', 'r') as f:
            config = json.load(f)
            robot_mode = config['robot_mode']

            if robot_mode == 'always_data_driven':
                always_data_driven = True

        print("always_data_driven: ", always_data_driven)
        keywordchecker_response = KeywordCheckerResponse()
        deictic, vec = self.check_for_deictic(gpt_response.response)

        keywordchecker_response.response = gpt_response.response
        if vec is None or always_data_driven:
            keywordchecker_response.deictic = False
            # set position to 0,0,0 as default
            keywordchecker_response.pos_x = 0
            keywordchecker_response.pos_y = 0
            keywordchecker_response.pos_z = 0
        else:
            keywordchecker_response.deictic = deictic
            # set position to the position of the object. We do not consider the z coordinate
            keywordchecker_response.pos_x = vec[0]
            keywordchecker_response.pos_y = vec[1]
            keywordchecker_response.pos_z = 0

        self.pub.publish(keywordchecker_response)

    def load_keywords_objects(self):
        keywords_objects_path = self.directory_path + '/config/keywords_objects.txt'
        with(open(keywords_objects_path, 'r')) as f:
            keywords = []
            vectors = []

            for line in f.readlines():
                keyword, x, y = line.split("+")
                keywords.append(keyword)
                # load vectors of the objects x and y coordinates
                vectors.append((float(x), float(y)))

        # extend keywords with plural, singular and different forms of spelling
        keywords_extend = []
        p = inflect.engine()
        p.classical(all=True)
        for idx, keyword in enumerate(keywords):
            variations = [keyword]

            plural = p.plural_noun(keyword)
            if plural:
                variations.append(plural)
            singular = p.singular_noun(keyword)
            if singular:
                variations.append(singular)

            keywords_extend.append((variations, vectors[idx]))

        self.keyword_objects = keywords_extend

    def check_for_deictic(self, string):
        keyword_list = []
        for tupel in self.keyword_objects:
            keyword_list.extend(tupel[0])

        c = count_keywords(string, keyword_list)
        if c == 0 or c >= 2:
            # if no keyword is found or more than 2 keywords are found we want to use the data driven approach
            # because we can not decide which object is meant
            return False, None
        elif count_keywords(string, ['shelf', 'left', 'right', 'back', 'entry', 'front']) == 0:
            # if no keyword for a direction is found we want to use the data driven approach
            return False, None
        else:  # deictic gesture
            # get first matched keyword
            for tupel in self.keyword_objects:
                curr_keywords = tupel[0]
                for key in curr_keywords:
                    if key.lower() in string.lower():
                        return True, tupel[1]

            # fallback option
            return True, None
        

if __name__ == '__main__':
    try:
        keyword_checker_node = KeywordCheckerNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Why did you kill me?")
    