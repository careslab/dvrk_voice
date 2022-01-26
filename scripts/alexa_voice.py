import logging
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String


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

    if(run == "start"):
        pub = rospy.Publisher('/assistant/autocamera/run', Bool, queue_size=1).publish(True)

    elif(run == "stop"):
        pub = rospy.Publisher('/assistant/autocamera/run', Bool, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub")
        pub.publish(False)

    say = 'Davinci has been turned on'
    return statement(say)

@ask.intent("TrackToolIntent", convert={'tool': String})
def autocamera_rub(tool):
    print("RunAutoCameraIntent" + str(tool))
    
    if(tool == "right"):
        pub = rospy.Publisher('/assistant/autocamera/track', Bool, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub")
        pub.publish("Right")

    elif(tool == "left"):
        pub = rospy.Publisher('/assistant/autocamera/track', String, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub")
        pub.publish("Left")

    elif(tool == "middle"):
        pub = rospy.Publisher('/assistant/autocamera/track', String, queue_size=1)
        while pub.get_num_connections() < 1:
            print("waiting for pub")
        pub.publish("Middle")

    say = 'Davinci has been turned on'
    return statement(say)


if __name__ == '__main__':
    rospy.init_node('dvrk_voice', anonymous=True)
    app.run(debug=True)