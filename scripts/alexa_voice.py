import logging
from typing import Mapping
# from urllib import request
from flask import Flask, render_template, request, jsonify
from flask_ask import Ask, statement, question, session
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
import json


app = Flask(__name__)
ask = Ask(app, "/")

logging.getLogger("flask_ask").setLevel(logging.DEBUG)


@ask.launch
def new_game():

    welcome_msg = render_template('welcome')

    return question(welcome_msg)


@ask.intent('RunAutoCameraIntent')
def autocamera_rub(run):
    print("RunAutoCameraIntent: " + str(run))
    print(run)

    data = request.get_json()
    run = str(data['request']['intent']['slots']['run']['slotValue']['resolutions']['resolutionsPerAuthority'][0]['values'][0]['value']['name']).lower()  # I'm NEVER doing this again!
    print(run)

    if(run == "start"):
        pub = rospy.Publisher('/assistant/autocamera/run', Bool, queue_size=1).publish(True)
        while pub.get_num_connections() < 1:
            print("waiting for pub start")
        pub.publish(True)

    elif(run == "stop"):
        pub = rospy.Publisher('/assistant/autocamera/run', Bool, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub stop")
        pub.publish(False)

    say = 'Davinci has been turned on'
    return statement(say)

@ask.intent("TrackToolIntent")
def autocamera_rub(tool):
    print("RunAutoCameraIntent" + str(tool))
    
    data = request.get_json()
    print(data, '\n')
    tool = str(data['request']['intent']['slots']['tool']['resolutions']['resolutionsPerAuthority'][0]['values'][0]['value']['name']).lower()  # I'm NEVER doing this again!
    # [0]['values'][0]['value']['name']
    print(tool)

    if(tool == "right"):
        pub = rospy.Publisher('/assistant/autocamera/track', Bool, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub right")
        pub.publish("Right")

    elif(tool == "left"):
        pub = rospy.Publisher('/assistant/autocamera/track', String, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub left")
        pub.publish("Left")

    elif(tool == "midpoint"):
        pub = rospy.Publisher('/assistant/autocamera/track', String, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub mid")
        pub.publish("Middle")

    say = 'Davinci has been turned on'
    return statement(say)



if __name__ == '__main__':
    rospy.init_node('dvrk_voice', anonymous=True)
    app.run(debug=True)